import assert from 'node:assert/strict'
import { test } from 'node:test'
import { StarPortPage, connectionProgress } from '../web/js/starport.js'

test('each transport exposes its actual active connection stage', () => {
  for (const [state, options, title] of [
    ['wifi_info', {}, 'Getting car Wi-Fi details'],
    ['joining_wifi', {}, 'Joining car Wi-Fi'],
    ['waiting_for_usb', { wired: true }, 'Waiting for your USB cable'],
    ['waiting_for_ipad', { app: 'HeadunitPad' }, 'Open HeadunitPad on your iPad'],
    ['waiting_for_tablet', { app: 'Open Headunit' }, 'Open Open Headunit on your tablet'],
  ]) {
    const result = connectionProgress({ state, running: true }, options)
    assert.equal(result.title, title)
    assert.equal(result.active, true)
  }
})
test('connection success, suspension and errors do not masquerade as work in progress', () => {
  for (const state of ['idle', 'streaming', 'suspended', 'error']) {
    assert.equal(connectionProgress({ state, running: state !== 'idle' }).active, false)
  }
  assert.equal(connectionProgress({ state: 'backoff', running: true, error: 'Car unavailable' }).detail, 'Car unavailable')
  assert.equal(connectionProgress({ state: 'backoff', running: false }).active, false)
  assert.equal(connectionProgress({ state: 'streaming' }).tone, 'success')
})
test('pairing approval advances to finishing and failures stop the spinner', () => {
  const steps = pairing => StarPortPage.computed.pairSteps.call({ pairing, adapterSetup: false })
  const confirming = steps({ active: true, state: 'pairing', receiver: {}, prompt: {} })
  assert.equal(confirming.find(step => step.status === 'active').label, 'Confirm the code on both screens')
  const approved = steps({ active: true, state: 'pairing', receiver: {}, approved: true })
  assert.equal(approved.find(step => step.status === 'active').label, 'Finish Bluetooth pairing')
  assert.equal(steps({ active: true, state: 'failed', error: 'Timed out' }).some(step => step.status === 'active'), false)
  assert(steps({ active: false, state: 'paired' }).every(step => step.status === 'done'))
})
