"""Actual local IPC timestamp/body transport, never physical clock proof."""
from openpilot.cereal import messaging


def test_current_message_factories_preserve_event_body_and_explicit_header():
  publisher = messaging.PubMaster(['carState'])
  subscriber = messaging.SubMaster(['carState'], poll='carState')
  for index in range(2):
    # This is a literal transport sentinel, not a fresh source/native lease.
    stamp = 1_234_567_890 + index
    event = messaging.new_message('carState', valid=True, logMonoTime=stamp)
    event.carState.vEgo = 12.5 + index
    publisher.send('carState', event)
    subscriber.update(1000)
    assert subscriber.updated['carState'] and subscriber.seen['carState'] and subscriber.valid['carState']
    assert subscriber.logMonoTime['carState'] == stamp
    assert subscriber['carState'].vEgo == 12.5 + index
    messaging.reset_context()  # Compatibility no-op must not break owned sockets.
