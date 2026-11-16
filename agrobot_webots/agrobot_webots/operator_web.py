#!/usr/bin/env python3
"""
Servidor HTTP da interface web do operador.

Serve a página (share/agrobot_webots/web/) e, só para leitura, a pasta
dados/ do projeto (CSVs das missões, fotos da câmera de aquisição), para o
operador baixar o que o robô gravou sem abrir o WSL. O comando do robô não
passa por aqui: a página fala com o ROS pelo rosbridge (WebSocket).

    GET /                     a página
    GET /api/config           {"rosbridge_port": ...}
    GET /api/dados?dir=<rel>  lista uma pasta de dados/ (JSON)
    GET /dados/<rel>          um arquivo de dados/

Sem senha: qualquer um que alcance a porta vê os dados e comanda o robô.
Na simulação (WSL), a porta só abre no próprio Windows (localhost).
"""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
from urllib.parse import parse_qs, unquote, urlparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


def inside(root, relative):
    """Caminho absoluto de relative dentro de root, ou None se escapar."""
    root = os.path.realpath(root)
    path = os.path.realpath(os.path.join(root, relative.lstrip('/')))
    if path != root and not path.startswith(root + os.sep):
        return None
    return path


def list_dir(root, relative, limit=2000):
    """Conteúdo de uma pasta de dados/, as mais novas primeiro."""
    path = inside(root, relative)
    if path is None or not os.path.isdir(path):
        return None
    entries = []
    with os.scandir(path) as items:
        for item in items:
            if item.name.startswith('.'):
                continue
            try:
                stat = item.stat()
            except OSError:
                continue
            entries.append({
                'name': item.name,
                'dir': item.is_dir(),
                'size': 0 if item.is_dir() else stat.st_size,
                'mtime': stat.st_mtime,
            })
    entries.sort(key=lambda e: (not e['dir'], -e['mtime']))
    return {'dir': relative.strip('/'), 'entries': entries[:limit],
            'total': len(entries)}


class Handler(SimpleHTTPRequestHandler):

    def __init__(self, *args, data_root, config, **kwargs):
        self.data_root = data_root
        self.config = config
        super().__init__(*args, **kwargs)

    def log_message(self, *args):
        pass   # sem uma linha no terminal a cada imagem

    def end_headers(self):
        # A página muda a cada build; sem cache, o navegador pega a nova.
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def send_json(self, data, status=200):
        body = json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == '/api/config':
            self.send_json(self.config)
            return
        if url.path == '/api/dados':
            relative = parse_qs(url.query).get('dir', [''])[0]
            listing = list_dir(self.data_root, relative)
            if listing is None:
                self.send_json({'error': 'pasta não encontrada'}, 404)
            else:
                self.send_json(listing)
            return
        if url.path.startswith('/dados/'):
            path = inside(self.data_root, unquote(url.path[len('/dados/'):]))
            if path is None or not os.path.isfile(path):
                self.send_error(404)
                return
            self.send_file(path)
            return
        super().do_GET()

    def send_file(self, path):
        with open(path, 'rb') as source:
            stat = os.fstat(source.fileno())
            self.send_response(200)
            self.send_header('Content-Type', self.guess_type(path))
            self.send_header('Content-Length', str(stat.st_size))
            self.end_headers()
            self.copyfile(source, self.wfile)


class OperatorWeb(Node):

    def __init__(self):
        super().__init__('operator_web')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('data_dir', '')
        self.declare_parameter('web_dir', '')
        web_dir = self.get_parameter('web_dir').value or os.path.join(
            get_package_share_directory('agrobot_webots'), 'web')
        data_root = self.get_parameter('data_dir').value or os.path.join(
            os.path.expanduser('~'), '.ros')
        os.makedirs(data_root, exist_ok=True)
        config = {
            'rosbridge_port': int(self.get_parameter('rosbridge_port').value),
        }
        handler = partial(Handler, directory=web_dir, data_root=data_root,
                          config=config)
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.server = ThreadingHTTPServer((host, port), handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Interface do operador: abra http://localhost:{port} no '
            f'navegador (dados de {data_root}).')

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
