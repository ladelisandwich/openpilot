import assert from "node:assert/strict"
import { test } from "node:test"
import { beginSave, finishSave, reportSave, saveFeedback, showDraftChange } from "../web/js/save-feedback.js"

test("save feedback only reports confirmed writes and ignores stale completions", async () => {
  await reportSave(async () => false)
  assert.equal(saveFeedback.message, "Could not save")
  await reportSave(async () => true)
  assert.equal(saveFeedback.message, "Saved")
  const stale = beginSave()
  const current = beginSave()
  finishSave(stale, true)
  assert.equal(saveFeedback.message, "Saving preferences…")
  finishSave(current, false)
  assert.equal(saveFeedback.message, "Could not save")
  showDraftChange()
  assert.equal(saveFeedback.message, "Unsaved changes")
})
