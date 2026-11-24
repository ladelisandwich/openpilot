import assert from 'node:assert/strict'
import { ReplayHistory, drawReplayTimeline } from '../web/js/replay-timeline.js'
import { ReplayClient } from '../web/js/replay.js'

const ownerId = 'a'.repeat(32), sessionId = 'b'.repeat(32)
const legend = Object.fromEntries(['chill', 'experimental', 'aol', 'error', 'bookmark'].map((kind, i) => [kind, {label: kind, rgb: [10 + i, 20, 30]}]))
const status = {start: 0, end: 10, seekRanges: [[0, 4], [6, 10]]}
const page = {schemaVersion: 1, ownerId, sessionId, rowOffset: 0, totalRows: 4, source: 'qlog', legend,
  rows: [[0, 4, 'chill'], [2, 2.00001, 'error']], nextCursor: 200}
const history = new ReplayHistory(ownerId, sessionId)
history.append(page, status)
assert.equal(history.rows, null, 'partial history is never displayed')
assert.equal(history.entry(1), null)
history.append({...page, rowOffset: 2, rows: [[2.000005, 2.000005, 'bookmark'], [6, 10, 'aol']], nextCursor: null}, status)
assert.equal(history.rows.length, 4)
assert.equal(history.label(2.000006), 'error', 'bookmark does not erase the alert')
assert.equal(history.entry(2.000006), legend.error)
assert.equal(history.label(5), 'Neutral')
assert.equal(history.entry(5), null)
assert.equal(history.label(6), 'aol')
for (const changes of [{sessionId: 'c'.repeat(32)}, {rowOffset: 1}, {rows: [[3, 7, 'chill']]},
  {rows: [[0, Infinity, 'error']]}, {nextCursor: 0}, {rows: [[0, 1, 'unknown']]}]) {
  assert.throws(() => new ReplayHistory(ownerId, sessionId).append({...page, ...changes}, status))
}
assert.throws(() => {
  const broken = new ReplayHistory(ownerId, sessionId)
  broken.append(page, status)
  broken.append({...page, rowOffset: 2, totalRows: 3, rows: [[6, 10, 'aol']], nextCursor: null}, status)
}, /History unavailable/, 'a late page cannot change the declared total')

const painted = []
let height = 31, tick = 5
globalThis.getComputedStyle = () => ({getPropertyValue: key => key === '--replay-tick' ? String(tick) : '2'})
const canvas = {getBoundingClientRect: () => ({width: 100, height}), getContext: () => ({
  setTransform() {}, set fillStyle(value) { this.color = value }, fillRect(x, y, w, h) { painted.push({x, y, w, h, color: this.color}) },
})}
drawReplayTimeline(canvas, status, history, 2)
assert.equal(canvas.width, 200)
assert.equal(canvas.height, 62)
const red = painted.findIndex(row => row.color === 'rgb(13 20 30)')
const mark = painted.findIndex(row => row.color === 'rgb(14 20 30)')
assert.ok(red > painted.findIndex(row => row.color === 'rgb(12 20 30)'), 'red is painted after all mode blocks')
assert.ok(mark > red && painted[mark].y < painted[red].y, 'bookmark occupies its own lane')
assert.ok(painted[red].w >= .5, 'a brief error occupies at least one physical pixel at DPR2')
assert.equal(painted[mark].w, 3, 'bookmark remains three CSS pixels wide at DPR2')
assert.equal(painted[red].h, 24)
height = 26; tick = 4
drawReplayTimeline(canvas, status, history, 1)
assert.equal(canvas.height, 26)
assert.equal(painted.at(-2).h, 20)
console.log('Replay timeline: atomic pages, validation, independent alerts, gaps, pixel precedence and compact sizing passed')

// A late page must not restore history after ownership or offroad authority disappears.
for (const change of [{owned: false}, {offroad: false}, {sessionId: 'c'.repeat(32)}]) {
  let release
  const response = new Promise(resolve => { release = resolve })
  const client = new ReplayClient({clientId: 'c'.repeat(32), publish() {}, unauthorized() {}, video: () => null,
    later: () => 1, cancelTimer() {}, fetcher: async () => response})
  client.active = true
  client.routeId = 'fixture'
  const state = {...status, ownerId, sessionId, routeId: 'fixture', revision: 1, epoch: 1, state: 'paused', paused: true,
    rate: 1, owned: true, offroad: true, timelineState: 'ready'}
  client.receive(state)
  assert.ok(client.timeline)
  client.receive({...state, revision: 2, ...change})
  const current = client.timeline
  if (change.sessionId) assert.equal(current.sessionId, change.sessionId)
  else assert.equal(current, null)
  release({ok: true, status: 200, json: async () => ({...page, totalRows: 2, nextCursor: null})})
  await new Promise(resolve => setImmediate(resolve))
  assert.equal(client.timeline, current)
  assert.equal(client.timeline?.rows ?? null, null)
  assert.equal(client.error, '')
  client.close(false)
}
console.log('Replay timeline: late metadata cannot survive ownership, session or offroad changes')
