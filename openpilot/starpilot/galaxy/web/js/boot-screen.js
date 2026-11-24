import { UPDATE_RESTART_MESSAGE } from "./update-restart.js"

const screen = document.getElementById("galaxy-boot")
const message = screen.querySelector("p")
const retry = screen.querySelector("button")
let mode = "opening"
let hideTimer = null

retry.addEventListener("click", () => location.reload())

export function bootScreenMode() { return mode }

export function showBootScreen(next, text, refresh = false) {
  clearTimeout(hideTimer)
  mode = next
  screen.hidden = false
  screen.classList.remove("gx-boot--done", "gx-boot--arriving")
  screen.classList.toggle("gx-boot--failed", next === "failed")
  message.textContent = text
  retry.hidden = !refresh
}

export function showRestartScreen() {
  const hidden = screen.hidden || screen.classList.contains("gx-boot--done")
  showBootScreen("restart", UPDATE_RESTART_MESSAGE, true)
  if (hidden) screen.classList.add("gx-boot--arriving")
}

export function dismissBootScreen() {
  if (mode === "hidden") return
  mode = "hidden"
  screen.classList.remove("gx-boot--arriving")
  if (matchMedia("(prefers-reduced-motion: reduce)").matches) { screen.hidden = true; return }
  screen.classList.add("gx-boot--done")
  hideTimer = setTimeout(() => { if (mode === "hidden") screen.hidden = true }, 500)
}
