const KINDS = ["chill", "experimental", "aol", "error", "bookmark"]
const finite = value => typeof value === "number" && Number.isFinite(value) && value >= 0 && value <= 604800
const layer = kind => ["error", "bookmark"].includes(kind) ? kind : "mode"

export class ReplayHistory {
  constructor(ownerId, sessionId) {
    Object.assign(this, { ownerId, sessionId, cursor: 0, pending: [], rows: null, ends: {}, previous: null, layers: {} })
  }
  append(page, status) {
    const fail = () => { throw new Error("History unavailable") }
    const integer = value => Number.isSafeInteger(value) && value >= 0
    const legend = page?.legend
    if (page?.schemaVersion !== 1 || page.ownerId !== this.ownerId || page.sessionId !== this.sessionId ||
      !integer(page.rowOffset) || page.rowOffset !== this.pending.length || !integer(page.totalRows) || page.totalRows > 65536 ||
      !["qlog", "rlog", "mixed"].includes(page.source) || !Array.isArray(page.rows) || page.rows.length > 256 ||
      !legend || Object.keys(legend).length !== KINDS.length || KINDS.some(kind =>
        typeof legend[kind]?.label !== "string" || legend[kind].label.length > 32 ||
        !Array.isArray(legend[kind].rgb) || legend[kind].rgb.length !== 3 || legend[kind].rgb.some(c => !integer(c) || c > 255)) ||
      this.total !== undefined && (page.totalRows !== this.total || page.source !== this.source || JSON.stringify(legend) !== JSON.stringify(this.legend))) fail()
    const ranges = status.seekRanges?.length ? status.seekRanges : [[status.start, status.end]]
    for (const row of page.rows) {
      if (!Array.isArray(row) || row.length !== 3 || !KINDS.includes(row[2]) || !row.slice(0, 2).every(finite) ||
        (row[2] === "bookmark" ? row[0] !== row[1] : row[0] >= row[1]) ||
        !ranges.some(([lo, hi]) => lo <= row[0] && row[1] <= hi)) fail()
      const prior = this.previous
      if (prior && (row[0] < prior[0] || row[0] === prior[0] && (row[1] < prior[1] ||
        row[1] === prior[1] && KINDS.indexOf(row[2]) <= KINDS.indexOf(prior[2])))) fail()
      const track = layer(row[2])
      if (row[0] < (this.ends[track] ?? -1) || track === "bookmark" && row[0] === this.ends[track]) fail()
      this.ends[track] = row[1]; this.previous = row
    }
    const count = this.pending.length + page.rows.length
    if (count > page.totalRows || (page.nextCursor === null) !== (count === page.totalRows) ||
      page.nextCursor !== null && (!integer(page.nextCursor) || page.nextCursor <= this.cursor || page.nextCursor >= 8 * 1024 * 1024) ||
      !page.rows.length && page.totalRows !== 0) fail()
    Object.assign(this, { total: page.totalRows, source: page.source, legend, cursor: page.nextCursor })
    this.pending.push(...page.rows)
    if (this.cursor === null) {
      this.rows = this.pending; this.pending = []
      for (const track of ["mode", "error"]) this.layers[track] = this.rows.filter(row => layer(row[2]) === track)
    }
  }
  entry(position) {
    if (!this.rows) return null
    for (const track of ["error", "mode"]) {
      const rows = this.layers[track]
      let lo = 0, hi = rows.length
      while (lo < hi) { const mid = (lo + hi) >>> 1; if (rows[mid][0] <= position) lo = mid + 1; else hi = mid }
      if (lo && position < rows[lo-1][1]) return this.legend[rows[lo-1][2]]
    }
    return null
  }
  label(position) { return this.entry(position)?.label || (this.rows ? "Neutral" : "Loading history") }
}

export function drawReplayTimeline(canvas, status, history, dpr = globalThis.devicePixelRatio || 1) {
  if (!canvas || !status) return
  const { width, height } = canvas.getBoundingClientRect(), style = getComputedStyle(canvas)
  const tick = parseFloat(style.getPropertyValue("--replay-tick")), y = tick + parseFloat(style.getPropertyValue("--replay-gap")), rail = height - y
  canvas.width = Math.max(1, Math.round(width * dpr)); canvas.height = Math.round(height * dpr)
  const ctx = canvas.getContext("2d")
  if (!ctx || status.end <= status.start) return
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  const rect = (left, right, color, bookmark = false) => {
    let x = Math.max(0, Math.min(width - 1 / dpr, width * (left - status.start) / (status.end - status.start)))
    const end = Math.min(width, Math.max(x + (bookmark ? 3 : 1 / dpr), width * (right - status.start) / (status.end - status.start)))
    if (bookmark) x = Math.max(0, end - 3)
    ctx.fillStyle = color; ctx.fillRect(x, bookmark ? 0 : y, end - x, bookmark ? tick : rail)
  }
  for (const [left, right] of status.seekRanges?.length ? status.seekRanges : [[status.start, status.end]]) rect(left, right, "#363c47")
  if (history?.rows) for (const track of ["mode", "error", "bookmark"]) for (const [left, right, kind] of history.rows) {
    if (layer(kind) === track) rect(left, right, `rgb(${history.legend[kind].rgb.join(" ")})`, track === "bookmark")
  }
}
