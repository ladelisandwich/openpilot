import assert from 'node:assert/strict'
import { ReplayExportClient, validReplayExport } from '../web/js/replay-export.js'
import { ReplayClient } from '../web/js/replay.js'
import { RouteWorkspace } from '../web/js/route-workspace.js'
import { validRouteDetail } from '../web/js/recording-data.js'

const route = '00000042--abcdef1234', clientId = '1'.repeat(32), exportId = '2'.repeat(32)
const idle = {schemaVersion:1,ownerId:'a'.repeat(32),sessionId:null,state:'idle',revision:0,epoch:0,
  position:0,start:0,end:0,rate:1,paused:true,offroad:true,owned:false}
const job = {id:exportId,routeId:route,segment:0,state:'rendering',progress:.2,bytes:0,filename:'route-replay-segment-0.mp4',owned:true}
assert.equal(validReplayExport(job), true)
for (const invalid of [{id:'../file'}, {filename:'../file.mp4'}, {progress:NaN}, {bytes:-1}, {segment:-1}, {routeId:'evil'}, {state:'unknown'}]) {
  assert.equal(validReplayExport({...job,...invalid}), false)
}
let status = idle, revoked = 0
const calls = [], timers = new Map()
let next = 0
const options = {clientId, unauthorized:()=>revoked++, later:(fn, ms)=>{timers.set(++next,{fn,ms});return next}, cancelTimer:id=>timers.delete(id),
  fetcher:async (path, options)=>{
    const body = options.body ? JSON.parse(options.body) : null
    calls.push({path,body})
    if (path.endsWith('/export-start')) status={...idle,sessionId:exportId,state:'preparing',purpose:'export',export:{...job,segment:body.segment ?? null}}
    if (path.endsWith('/stop')) status={...idle,export:{...status.export,state:'cancelled'}}
    return {ok:true,status:200,json:async()=>status}
  }}
const exports = new ReplayExportClient(options)
await exports.open()
assert.equal(exports.running, false)
let prepared = false
await exports.start(route, 0, async()=>{prepared=true})
assert.equal(prepared, true)
assert.equal(calls.find(call=>call.path.endsWith('/export-start')).body.segment, 0, 'segment zero is never interpreted as route export')
assert.equal(exports.running, true)
assert.equal(exports.downloadUrl, null, 'no download link before finalized output')
exports.close()
assert.equal(calls.filter(call=>call.path.endsWith('/stop')).length, 0, 'leaving the workspace never cancels an export')
const reconnected = new ReplayExportClient(options)
await reconnected.open()
assert.equal(reconnected.job.id, exportId, 'refresh reconnects to the same authenticated receipt')
status = {...idle,export:{...job,state:'ready',bytes:12500}}
await reconnected.refresh()
assert.equal(reconnected.running, false)
assert.match(reconnected.downloadUrl, /export-download\?exportId=/)
assert.doesNotMatch(reconnected.downloadUrl, /recordings\/media/)
status = {...idle,export:{...job,segment:null}}
await reconnected.refresh()
await reconnected.cancel()
assert.equal(reconnected.job.state, 'cancelled')
assert.equal(calls.find(call=>call.path.endsWith('/stop')).body.sessionId, exportId)
reconnected.close()
const denied = new ReplayExportClient({...options,fetcher:async()=>({ok:false,status:401,json:async()=>({error:'Sign in'})})})
await denied.open()
assert.equal(revoked, 1)
assert.equal(denied.active, false)
assert.equal(timers.size, 0, 'authentication failure stops polling')

let offline = true
const retrying = new ReplayExportClient({...options,fetcher:async()=>{
  if (offline) throw Error('Temporary disconnect')
  return {ok:true,status:200,json:async()=>status}
}})
await retrying.open()
assert.equal(retrying.job, null)
assert.match(retrying.error, /Temporary disconnect/)
assert.equal([...timers.values()][0].ms, 1000, 'an initial failure retries before a job is known')
offline = false; status = {...idle,export:job}
await [...timers.values()][0].fn()
assert.equal(retrying.job.id, exportId)
assert.equal(retrying.error, '')
status = {...idle,export:{...job,state:'ready',bytes:12500}}
await [...timers.values()][0].fn()
assert.equal([...timers.values()][0].ms, 1800000, 'ready exports retain their expiry timer')
offline = true
await retrying.refresh()
assert.equal([...timers.values()][0].ms, 1000, 'a failed ready-status refresh retries promptly')
offline = false; status = idle
await [...timers.values()][0].fn()
assert.equal(timers.size, 0, 'a successful idle response needs no polling')
offline = true
await retrying.refresh()
retrying.close()
assert.equal(timers.size, 0, 'leaving the page cancels retries')

let failStatus
const leaving = new ReplayExportClient({...options,fetcher:()=>new Promise((resolve,reject)=>{failStatus=reject})})
const opening = leaving.open()
leaving.close()
failStatus(Error('Disconnected after leaving'))
await opening
assert.equal(leaving.error, '')
assert.equal(timers.size, 0, 'an in-flight failure cannot restart polling after navigation or logout')

// A complete route may exceed the list's 64-segment summary without losing its last files.
const segments = Array.from({length:100},(_,number)=>({number,segmentName:`${route}--${number}`,
  files:{rlog:true,qlog:false,fcamera:false,dcamera:false,ecamera:false,qcamera:false}}))
assert.equal(validRouteDetail({schemaVersion:1,source:'local',route:{routeId:route,inventoryComplete:true,segmentCount:100,segments}},route),true)
const unavailable = {...segments[1],files:Object.fromEntries(Object.keys(segments[1].files).map(key=>[key,false]))}
const incomplete = {schemaVersion:1,source:'local',route:{routeId:route,inventoryComplete:false,segmentCount:2,
  segments:[segments[0],unavailable]}}
assert.equal(validRouteDetail(incomplete,route),true,'an unavailable tail cannot hide closed segment downloads')
assert.equal(validRouteDetail({...incomplete,route:{...incomplete.route,segments:[segments[0],
  {...unavailable,files:{...unavailable.files,rlog:'false'}}]}},route),false,'file flags must remain booleans')
const workspace={...RouteWorkspace.data.call({initialMode:'replay'}),route:{segments},$refs:{},exports:{running:false}}
Object.assign(workspace,RouteWorkspace.methods)
workspace.closeDetails=()=>{}
workspace.segmentNumber=0
await workspace.chooseSegment(99)
assert.equal(workspace.segmentNumber,99)

// A failed stop cannot switch modes or begin an export over an existing renderer.
workspace.$refs.replay={stopAndWait:async()=>{throw Error('still stopping')}}
await workspace.changeMode('footage')
assert.equal(workspace.viewMode,'replay')
assert.match(workspace.error,/still stopping/)
console.log('Replay export: mode-specific actions, exact scope, reconnect, cancellation, complete inventory and failed-stop guards passed')

// A refreshed tab can stop its orphaned preview before starting its export.
status = {...idle,owned:true,sessionId:'b'.repeat(32),routeId:route,state:'paused',purpose:'view'}
const restored = new ReplayExportClient(options)
await restored.open()
const firstNewCall = calls.length
await restored.start(route)
const newActions = calls.slice(firstNewCall).filter(call=>call.body)
assert.equal(newActions[0].path, './api/replay/stop')
assert.equal(newActions[1].path, './api/replay/export-start')
restored.close()

const uncertain = new ReplayClient({clientId,video:()=>null,publish(){},unauthorized(){}})
uncertain.active=true; uncertain.ownedSession=exportId; uncertain.status={...idle,sessionId:exportId,state:'stopping'}
uncertain.stop=async()=>{}
const now = performance.now
let clock = 0
performance.now=()=>clock+=10000
try { await assert.rejects(uncertain.stopAndWait(), /still stopping/) }
finally { performance.now=now; uncertain.close(false) }

restored.receive({...idle,export:{...job,state:'ready',expiresInSeconds:0}})
assert.equal(restored.downloadUrl,null,'expired exports cannot offer a stale download')
