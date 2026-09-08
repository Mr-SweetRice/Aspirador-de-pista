export class CommandCancelledError extends Error {
  constructor() { super('Comando cancelado por parada ou desconexão'); }
}

type Entry = {
  packet: Uint8Array;
  epoch: number;
  resolve: () => void;
  reject: (error: unknown) => void;
};

/** A single in-flight ATT operation; cancellation never creates a second writer. */
export class CommandQueue {
  generation = 0;
  private pending: Entry[] = [];
  private running = false;

  cancel() {
    this.generation++;
    for (const entry of this.pending.splice(0)) entry.reject(new CommandCancelledError());
  }

  constructor(private write: (packet: Uint8Array, current: () => boolean) => Promise<void>) {}

  enqueue(packet: Uint8Array, stop = false): Promise<void> {
    if (stop) this.cancel();
    if (this.pending.length >= 128) return Promise.reject(new Error('Fila BLE cheia'));
    const result = new Promise<void>((resolve, reject) => {
      this.pending.push({ packet: packet.slice(), epoch: this.generation, resolve, reject });
    });
    void this.drain();
    return result;
  }

  private async drain() {
    if (this.running) return;
    this.running = true;
    try {
      while (this.pending.length) {
        const entry = this.pending.shift()!;
        const current = () => entry.epoch === this.generation;
        try {
          if (!current()) throw new CommandCancelledError();
          await this.write(entry.packet, current);
          if (!current()) throw new CommandCancelledError();
          entry.resolve();
        } catch (error) {
          entry.reject(error);
          // Preserve a STOP queued while the previous write was failing.
          if (current()) this.cancel();
        }
      }
    } finally {
      this.running = false;
    }
  }
}
