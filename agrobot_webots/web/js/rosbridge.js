// Cliente mínimo do protocolo do rosbridge (JSON por WebSocket), sem
// dependências: inscrever, publicar e chamar serviços. Reconecta sozinho e
// refaz as inscrições e os anúncios.

export class Ros {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.connected = false;
    this.nextId = 1;
    this.subscriptions = new Map();   // id -> {topic, type, options, callback}
    this.advertised = new Map();      // topic -> type
    this.pending = new Map();         // id -> {resolve, reject, timer}
    this.listeners = [];
    this.connect();
  }

  onConnection(callback) { this.listeners.push(callback); }

  emit(state) { this.listeners.forEach((cb) => cb(state)); }

  connect() {
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch (error) {
      setTimeout(() => this.connect(), 2000);
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.connected = true;
      for (const [topic, type] of this.advertised) {
        this.send({ op: 'advertise', topic, type });
      }
      for (const [id, sub] of this.subscriptions) this.sendSubscribe(id, sub);
      this.emit(true);
    };
    ws.onclose = () => {
      const was = this.connected;
      this.connected = false;
      for (const [, call] of this.pending) {
        clearTimeout(call.timer);
        call.reject(new Error('conexão perdida'));
      }
      this.pending.clear();
      if (was) this.emit(false);
      setTimeout(() => this.connect(), 2000);
    };
    ws.onerror = () => {};
    ws.onmessage = (event) => this.receive(JSON.parse(event.data));
  }

  send(message) {
    if (this.connected) this.ws.send(JSON.stringify(message));
  }

  receive(message) {
    if (message.op === 'publish') {
      for (const [, sub] of this.subscriptions) {
        if (sub.topic === message.topic) sub.callback(message.msg);
      }
    } else if (message.op === 'service_response') {
      const call = this.pending.get(message.id);
      if (!call) return;
      this.pending.delete(message.id);
      clearTimeout(call.timer);
      if (message.result === false) call.reject(new Error(String(message.values)));
      else call.resolve(message.values);
    }
  }

  sendSubscribe(id, sub) {
    this.send({ op: 'subscribe', id, topic: sub.topic, type: sub.type, ...sub.options });
  }

  // options: throttle_rate (ms entre mensagens), queue_length.
  subscribe(topic, type, callback, options = {}) {
    const id = `sub_${this.nextId++}`;
    const sub = { topic, type, callback, options };
    this.subscriptions.set(id, sub);
    this.sendSubscribe(id, sub);
    return () => {
      this.subscriptions.delete(id);
      this.send({ op: 'unsubscribe', id, topic });
    };
  }

  publish(topic, type, msg) {
    if (!this.advertised.has(topic)) {
      this.advertised.set(topic, type);
      this.send({ op: 'advertise', topic, type });
    }
    this.send({ op: 'publish', topic, msg });
  }

  callService(service, type, args = {}, timeoutMs = 5000) {
    return new Promise((resolve, reject) => {
      if (!this.connected) {
        reject(new Error('sem conexão com o robô'));
        return;
      }
      const id = `call_${this.nextId++}`;
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`${service} não respondeu`));
      }, timeoutMs);
      this.pending.set(id, { resolve, reject, timer });
      this.send({ op: 'call_service', id, service, type, args });
    });
  }
}
