import assert from 'node:assert/strict';
import test from 'node:test';
import { CommandQueue, CommandCancelledError } from './CommandQueue';

test('STOP cancels pending PWM/start and keeps only one ATT write in flight', async () => {
  const sent: number[] = [];
  let release!: () => void;
  const gate = new Promise<void>((resolve) => { release = resolve; });
  const queue = new CommandQueue(async (packet) => {
    sent.push(packet[0]!);
    if (sent.length === 1) await gate;
  });
  const first = queue.enqueue(new Uint8Array([10]));
  const pending = queue.enqueue(new Uint8Array([11]));
  const cancelledFirst = assert.rejects(first, CommandCancelledError);
  const cancelledPending = assert.rejects(pending, CommandCancelledError);
  const stop = queue.enqueue(new Uint8Array([1]), true);
  assert.deepEqual(sent, [10]);
  release();
  await Promise.all([cancelledFirst, cancelledPending, stop]);
  assert.deepEqual(sent, [10, 1]);
});

test('configuration failure cancels queued START', async () => {
  const sent: number[] = [];
  const queue = new CommandQueue(async (packet) => {
    sent.push(packet[0]!);
    throw new Error('ATT failure');
  });
  const config = queue.enqueue(new Uint8Array([42]));
  const start = queue.enqueue(new Uint8Array([44]));
  await Promise.all([assert.rejects(config, /ATT failure/), assert.rejects(start, CommandCancelledError)]);
  assert.deepEqual(sent, [42]);
});

test('cancel invalidates a sequence generation even with no pending commands', () => {
  const queue = new CommandQueue(async () => undefined);
  const epoch = queue.generation;
  queue.cancel();
  assert.notEqual(queue.generation, epoch);
});

test('successful commands retain order', async () => {
  const sent: number[] = [];
  const queue = new CommandQueue(async (packet) => { sent.push(packet[0]!); });
  await Promise.all([1, 2, 3].map((id) => queue.enqueue(new Uint8Array([id]))));
  assert.deepEqual(sent, [1, 2, 3]);
});
