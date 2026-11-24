const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright')
const assert = require('node:assert/strict')

;(async () => {
  const browser = await chromium.launch({ headless: true })
  const page = await browser.newPage({ deviceScaleFactor: 2, hasTouch: true })
  const errors = []
  page.on('pageerror', error => errors.push(error.message))
  await page.route('**/js/boot.js', route => route.fulfill({ body: '' }))
  await page.route('**/api/**', route => route.fulfill({ status: 404, body: '' }))
  await page.goto(process.env.GALAXY_URL || 'http://127.0.0.1:8765/')
  await page.evaluate(async () => {
    const { createApp, markRaw } = await import('./vendor/vue/vue.esm-browser.js')
    const { ReplayViewer } = await import('./js/replay.js')
    const { ReplayHistory } = await import('./js/replay-timeline.js')
    const ownerId = 'a'.repeat(32), sessionId = 'b'.repeat(32)
    const status = { schemaVersion: 1, ownerId, sessionId, state: 'paused', owned: true, offroad: true,
      epoch: 1, start: 0, end: 120, position: 60, paused: true, rate: 1, seekRanges: [[0, 70], [80, 120]], sourceWarnings: [] }
    const legend = Object.fromEntries([
      ['chill', 'Chill', [22, 127, 64]], ['experimental', 'Experimental', [218, 111, 37]], ['aol', 'AOL', [10, 186, 181]],
      ['error', 'Alert', [201, 34, 49]], ['bookmark', 'Bookmark', [255, 214, 0]],
    ].map(([kind, label, rgb]) => [kind, { label, rgb }]))
    const history = new ReplayHistory(ownerId, sessionId)
    history.append({ schemaVersion: 1, ownerId, sessionId, rowOffset: 0, totalRows: 7, nextCursor: null, source: 'qlog', legend,
      rows: [[0, 20, 'chill'], [20, 40, 'aol'], [40, 70, 'experimental'], [55, 56, 'error'],
        [55.5, 55.5, 'bookmark'], [80, 120, 'chill'], [100, 100, 'bookmark']] }, status)
    document.body.innerHTML = `<main class="gx-content"><div id="workspace" class="gx-route-workspace">
      <div class="gx-route-workspace__body"><section class="gx-card gx-route-workspace__viewer"><div id="fixture"></div></section>
      <aside class="gx-card gx-route-workspace__files"><h3>Logs &amp; files</h3><p>Segment 0</p></aside></div></div></main>`
    window.replayControls = []
    window.fixtureVM = createApp({ ...ReplayViewer,
      data: () => ({ ...ReplayViewer.data(), replay: status, replayTimeline: markRaw(history), replayVideoReady: true,
        client: markRaw({ ownedSession: sessionId, control(...args) { window.replayControls.push(args) } }) }),
      mounted() {
        this.resizeRail = () => this.drawRail(true)
        this.railObserver = new ResizeObserver(this.resizeRail)
        this.railObserver.observe(this.$refs.player)
        window.addEventListener('resize', this.resizeRail)
        this.$nextTick(() => this.drawRail(true))
      },
      beforeUnmount() {},
    }, { clientId: 'c'.repeat(32), routeId: 'fixture', unauthorized() {} }).mount('#fixture')
    window.loadLongReplay = () => {
      const vm = window.fixtureVM, status = { ...vm.replay, end: 1968, position: 5, seekRanges: [[0, 1968]] }
      const rows = Array.from({ length: 328 }, (_, i) => [i * 6, (i + 1) * 6, i % 3 ? 'chill' : 'aol'])
      const history = new ReplayHistory(status.ownerId, status.sessionId)
      for (let offset = 0; offset < rows.length; offset += 256) history.append({ schemaVersion: 1, ownerId: status.ownerId,
        sessionId: status.sessionId, rowOffset: offset, totalRows: rows.length, source: 'qlog', legend,
        rows: rows.slice(offset, offset + 256), nextCursor: offset + 256 < rows.length ? offset + 256 : null }, status)
      vm.replay = status; vm.replayTimeline = markRaw(history)
    }
  })
  const geometry = async () => page.locator('.gx-replay__media-controls').evaluate(media => {
    const bounds = element => {
      const { x, y, width, height } = element.getBoundingClientRect()
      return { x, y, width, height }
    }
    const track = media.querySelector('.gx-replay__track'), canvas = track.querySelector('canvas'), style = getComputedStyle(media)
    return { media: bounds(media), caption: bounds(media.querySelector('.gx-replay__timeline > span')),
      track: bounds(track), canvas: bounds(canvas), input: bounds(track.querySelector('input')),
      controls: bounds(media.querySelector('.gx-replay__controls')), pixels: canvas.width,
      available: media.clientWidth - parseFloat(style.paddingLeft) - parseFloat(style.paddingRight),
      overflow: document.documentElement.scrollWidth > innerWidth }
  })
  const assertLayout = async width => {
    await page.waitForFunction(width => {
      const canvas = document.querySelector('.gx-replay__rail canvas')
      return canvas?.height === (width <= 600 ? 52 : 62) && Math.abs(canvas.width - canvas.getBoundingClientRect().width * 2) <= 1
    }, width)
    const g = await geometry()
    assert.equal(g.canvas.height, width <= 600 ? 26 : 31)
    assert.equal(g.input.height, 48)
    assert.ok(Math.abs(g.track.width - g.available) <= 1, 'timeline uses the full control area')
    assert.ok(Math.abs(g.canvas.width - (g.track.width - 16)) <= 1)
    assert.ok(g.track.y >= g.caption.y + g.caption.height - 1, 'caption is above the track')
    assert.ok(g.controls.y >= g.track.y + g.track.height - 1, 'buttons are below the track')
    assert.ok(Math.abs(g.pixels - g.canvas.width * 2) <= 1)
    assert.equal(g.overflow, false)
  }
  for (const [embedded, width] of [[false, 1280], [true, 1280], [true, 1000], [true, 390]]) {
    await page.evaluate(embedded => {
      const workspace = document.getElementById('workspace'), body = workspace.firstElementChild, viewer = body.firstElementChild
      workspace.className = embedded ? 'gx-route-workspace' : ''
      body.className = embedded ? 'gx-route-workspace__body' : ''
      viewer.className = embedded ? 'gx-card gx-route-workspace__viewer' : ''
      body.lastElementChild.hidden = !embedded
    }, embedded)
    await page.setViewportSize({ width, height: 900 })
    await assertLayout(width)
    if (embedded && width === 1280) {
      const viewer = await page.locator('.gx-route-workspace__viewer').boundingBox()
      const files = await page.locator('.gx-route-workspace__files').boundingBox()
      assert.ok(files.x >= viewer.x + viewer.width, 'desktop fixture includes the real files sidebar')
    }
    const slider = page.getByRole('slider', { name: 'Replay route position' })
    const box = await slider.boundingBox()
    const x = box.x + box.width * .5, y = box.y + box.height / 2
    const bitmap = await page.locator('.gx-replay__rail canvas').evaluate(canvas => canvas.toDataURL())
    await page.evaluate(() => {
      window.replayControls = []; window.replayPaints = 0
      const ctx = document.querySelector('.gx-replay__rail canvas').getContext('2d')
      if (!ctx.originalFillRect) {
        ctx.originalFillRect = ctx.fillRect
        ctx.fillRect = function (...args) { window.replayPaints++; this.originalFillRect(...args) }
      }
    })
    await page.mouse.move(x, y)
    await page.mouse.down()
    await page.mouse.move(box.x + box.width * .55, y + 25)
    const preview = page.locator('.gx-replay__preview')
    await preview.waitFor()
    assert.match(await preview.textContent(), /Experimental/)
    assert.equal(await page.evaluate(() => window.replayControls.length), 0, 'dragging only previews')
    const bounds = await preview.boundingBox()
    assert.ok(bounds.y + bounds.height < box.y - 20, 'preview stays above the touch target')
    assert.equal(await page.locator('.gx-replay__rail canvas').evaluate(canvas => canvas.toDataURL()), bitmap, 'scrubbing reuses the canvas')
    assert.equal(await page.evaluate(() => window.replayPaints), 0, 'scrubbing never repaints the history')
    if (process.env.REPLAY_SCREENSHOT_DIR) await page.screenshot({ path: `${process.env.REPLAY_SCREENSHOT_DIR}/replay-web-scrub-${embedded ? 'workspace' : 'standalone'}-${width}.png` })
    await page.mouse.up()
    assert.equal(await preview.count(), 0)
    assert.equal(await page.evaluate(() => window.replayControls.length), 1, 'release commits exactly once')
    for (const fraction of [0, 1]) {
      await page.mouse.move(box.x + 8 + (box.width - 16) * fraction, y)
      await page.mouse.down()
      const edge = await preview.boundingBox()
      const player = await page.locator('.gx-replay__player').boundingBox()
      assert.ok(edge.x >= player.x && edge.x + edge.width <= player.x + player.width, 'edge preview stays inside player')
      const head = await page.locator('.gx-replay__playhead').boundingBox()
      const stem = await page.locator('.gx-replay__preview-line').boundingBox()
      assert.ok(Math.abs(stem.x - (head.x + head.width / 2)) <= 1, 'stem still points to the target when the pill is clamped')
      await page.mouse.up()
    }
    for (const cancel of ['escape', 'epoch', 'session', 'offroad']) {
      await page.evaluate(() => { window.replayControls = [] })
      await page.mouse.move(x, y); await page.mouse.down()
      if (cancel === 'escape') await page.keyboard.press('Escape')
      else await page.evaluate(cancel => {
        const vm = window.fixtureVM
        if (cancel === 'epoch') vm.replay = { ...vm.replay, epoch: vm.replay.epoch + 1 }
        if (cancel === 'session') {
          const sessionId = vm.replay.sessionId === 'b'.repeat(32) ? 'd'.repeat(32) : 'b'.repeat(32)
          vm.client.ownedSession = sessionId
          vm.replay = { ...vm.replay, sessionId }
        }
        if (cancel === 'offroad') vm.replay = { ...vm.replay, offroad: false }
      }, cancel)
      await page.mouse.move(box.x + box.width * .6, y)
      await page.mouse.up()
      assert.equal(await preview.count(), 0, `${cancel} clears the preview`)
      assert.equal(await page.evaluate(() => window.replayControls.length), 0, `${cancel} cancels the seek`)
      await page.evaluate(() => { window.fixtureVM.replay = { ...window.fixtureVM.replay, offroad: true } })
    }
    await slider.focus()
    await page.keyboard.press('ArrowRight')
    assert.match(await slider.getAttribute('aria-valuetext'), /Experimental/)
    await page.locator('summary', { hasText: 'Timeline colors' }).click()
    assert.equal(await page.locator('.gx-replay__legend span').count(), 5)
    if (process.env.REPLAY_SCREENSHOT_DIR) await page.screenshot({ path: `${process.env.REPLAY_SCREENSHOT_DIR}/replay-web-${embedded ? 'workspace' : 'standalone'}-${width}.png` })
    await page.locator('summary', { hasText: 'Timeline colors' }).click()
    await page.getByRole('button', { name: 'Fullscreen video' }).click()
    await page.waitForFunction(() => document.fullscreenElement)
    await assertLayout(width)
    const fullscreenHeight = await page.locator('.gx-replay__track canvas').evaluate(canvas => ({
      actual: canvas.getBoundingClientRect().height, expected: innerWidth <= 600 ? 26 : 31,
    }))
    assert.equal(fullscreenHeight.actual, fullscreenHeight.expected, 'fullscreen preserves restrained rail sizing')
    if (process.env.REPLAY_SCREENSHOT_DIR) await page.screenshot({ path: `${process.env.REPLAY_SCREENSHOT_DIR}/replay-web-fullscreen-${embedded ? 'workspace' : 'standalone'}-${width}.png` })
    await page.evaluate(() => document.exitFullscreen())
  }
  await page.setViewportSize({ width: 390, height: 900 })
  const slider = page.getByRole('slider', { name: 'Replay route position' })
  const box = await slider.boundingBox(), cdp = await page.context().newCDPSession(page)
  for (const cancel of [false, true]) {
    await page.evaluate(() => { window.replayControls = [] })
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: box.x + box.width * .5, y: box.y + box.height / 2 }] })
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: box.x + box.width * .6, y: box.y + box.height / 2 + 20 }] })
    assert.equal(await page.locator('.gx-replay__preview').count(), 1, 'touch drag displays the raised preview')
    assert.equal(await page.evaluate(() => window.replayControls.length), 0)
    await cdp.send('Input.dispatchTouchEvent', { type: cancel ? 'touchCancel' : 'touchEnd', touchPoints: [] })
    assert.equal(await page.locator('.gx-replay__preview').count(), 0)
    assert.equal(await page.evaluate(() => window.replayControls.length), cancel ? 0 : 1, 'touch end commits once; cancellation never commits')
  }
  await page.evaluate(() => window.loadLongReplay())
  for (const width of [1280, 390, 320]) {
    await page.setViewportSize({ width, height: 900 })
    await assertLayout(width)
    assert.match(await page.locator('.gx-replay__timeline').textContent(), /32:48/)
    if (process.env.REPLAY_SCREENSHOT_DIR) await page.screenshot({ path: `${process.env.REPLAY_SCREENSHOT_DIR}/replay-long-route-${width}.png` })
  }
  await page.evaluate(() => {
    window.fixtureVM.replayTimeline = null
    window.fixtureVM.replayHistoryNotice = 'History unavailable; the recorded route can still be replayed.'
  })
  await assertLayout(320)
  assert.deepEqual(errors, [])
  await browser.close()
  console.log('Replay timeline browser: embedded/standalone layout, dense long routes, cached dragging, touch, cancellation, fullscreen and keyboard passed')
})().catch(error => { console.error(error); process.exit(1) })
