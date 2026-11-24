import { reactive } from "../vendor/vue/vue.esm-browser.js"

export const saveFeedback = reactive({ visible: false, message: "" })
let generation = 0
let timer

export function beginSave() {
  clearTimeout(timer)
  saveFeedback.message = "Saving preferences…"
  saveFeedback.visible = true
  const token = ++generation
  timer = setTimeout(() => finishSave(token, false), 12000)
  return token
}

export function finishSave(token, saved) {
  if (token !== generation) return
  clearTimeout(timer)
  saveFeedback.message = saved ? "Saved" : "Could not save"
  timer = setTimeout(() => { if (token === generation) saveFeedback.visible = false }, 1400)
}

export function showDraftChange() {
  clearTimeout(timer)
  const token = ++generation
  saveFeedback.message = "Unsaved changes"
  saveFeedback.visible = true
  timer = setTimeout(() => { if (token === generation) saveFeedback.visible = false }, 1400)
}

export async function reportSave(operation) {
  const token = beginSave()
  try {
    const result = await operation()
    finishSave(token, !!result)
    return result
  } catch (error) {
    finishSave(token, false)
    throw error
  }
}

export const SaveFeedback = {
  setup: () => ({ saveFeedback }),
  template: `<p class="gx-settings__save-status gx-settings__save-status--animated" :class="{'is-visible': saveFeedback.visible}"
    :aria-hidden="!saveFeedback.visible" role="status">{{ saveFeedback.message }}</p>`,
}
