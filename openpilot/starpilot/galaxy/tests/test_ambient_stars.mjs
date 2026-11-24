import assert from "node:assert/strict"
import { readFile } from "node:fs/promises"
import { test } from "node:test"
import { spawnAmbientStars, pauseStarsOnSlowFirefoxScroll } from "../web/js/ambient-stars.js"

test("ambient stars populate one lightweight texture per star", () => {
  const children = []
  const background = {
    ownerDocument: { createElement: (tag) => {
      const style = {}
      Object.defineProperty(style, "setProperty", { value: (name, value) => { style[name] = value } })
      return { tag, style }
    } },
    querySelector: () => children.find((child) => child.className === "galaxy-hero") || null,
    appendChild: (child) => children.push(child),
  }
  assert.equal(spawnAmbientStars(background, () => 0.5), 14)
  assert.equal(children.length, 14)
  assert.deepEqual(children[0], { tag: "i", className: "galaxy-hero",
    style: { left: "50.00%", top: "50.00%", animationDelay: "2.00s", "--galaxy-star-core": "2px" } })
  assert.equal(spawnAmbientStars(background, () => 0), 0)
  assert.equal(children.length, 14)
  assert.equal(spawnAmbientStars(null), 0)
})
test("ambient stars use a stable texture without promoting the static backdrop", async () => {
  const css = await readFile(new URL("../web/css/galaxy.css", import.meta.url), "utf8")
  const backdrop = css.slice(css.indexOf("#galaxy-bg {"), css.indexOf("#galaxy-bg::before"))
  const starField = css.slice(css.indexOf("#galaxy-bg::before"), css.indexOf("#galaxy-bg .galaxy-hero"))
  const star = css.slice(css.indexOf("#galaxy-bg .galaxy-hero {"), css.indexOf("@keyframes galaxy-twinkle"))

  assert.doesNotMatch(backdrop, /will-change|translateZ/)
  assert.match(starField, /inset: 0/)
  assert.doesNotMatch(starField, /height:\s*200%/)
  assert.match(star, /radial-gradient/)
  assert.doesNotMatch(star, /box-shadow/)
  assert.match(css, /rotateZ\(0\.05deg\) scale\(0\.9\)/)
  assert.match(css, /rotateZ\(0\.05deg\) scale\(1\.2\)/)
  assert.match(css, /body:has\([^)]*\.gx-route-workspace[^)]*\) #galaxy-bg \.galaxy-hero\s*{\s*display: none;/)
  assert.match(css, /\.galaxy-stars-paused \.galaxy-hero\s*{\s*animation-play-state: paused;/)
})

test("slow scrolling pauses stars only on touch Firefox", () => {
  const originals = Object.fromEntries(["navigator", "matchMedia", "document", "window", "performance", "requestAnimationFrame"]
    .map((key) => [key, Object.getOwnPropertyDescriptor(globalThis, key)]))
  const listeners = new Map()
  const frames = []
  const classes = new Set()
  const background = { classList: { add: (name) => classes.add(name) } }
  let now = 0
  let routeOpen = false
  try {
    Object.defineProperties(globalThis, {
      navigator: { configurable: true, value: { userAgent: "Firefox/120 Android" } },
      matchMedia: { configurable: true, value: () => ({ matches: true }) },
      document: { configurable: true, value: { hidden: false, querySelector: () => routeOpen } },
      window: { configurable: true, value: {
        addEventListener: (name, callback) => listeners.set(name, callback),
        removeEventListener: (name) => listeners.delete(name),
      } },
      performance: { configurable: true, value: { now: () => now } },
      requestAnimationFrame: { configurable: true, value: (callback) => { frames.push(callback); return frames.length } },
    })
    Object.defineProperty(globalThis, "matchMedia", { configurable: true, value: () => ({ matches: false }) })
    pauseStarsOnSlowFirefoxScroll(background)
    assert.ok(!listeners.has("scroll"))
    Object.defineProperty(globalThis, "matchMedia", { configurable: true, value: () => ({ matches: true }) })
    Object.defineProperty(globalThis, "navigator", { configurable: true, value: { userAgent: "Firefox/120 desktop" } })
    pauseStarsOnSlowFirefoxScroll(background)
    assert.ok(!listeners.has("scroll"))
    Object.defineProperty(globalThis, "navigator", { configurable: true, value: { userAgent: "Android Firefox/120" } })
    pauseStarsOnSlowFirefoxScroll(background)
    assert.ok(listeners.has("scroll"))
    routeOpen = true
    listeners.get("scroll")()
    assert.equal(frames.length, 0)
    routeOpen = false
    listeners.get("scroll")()
    for (let index = 0; index < 65; index++) {
      now += 16
      frames.shift()(now)
      listeners.get("scroll")()
    }
    assert.ok(!classes.has("galaxy-stars-paused"))
    for (let index = 1; index <= 27; index++) {
      now += 40
      frames.shift()(now)
      listeners.get("scroll")?.()
      if (classes.has("galaxy-stars-paused")) break
    }
    assert.ok(classes.has("galaxy-stars-paused"))
    assert.ok(!listeners.has("scroll"))
  } finally {
    for (const [key, descriptor] of Object.entries(originals)) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor)
      else delete globalThis[key]
    }
  }
})
