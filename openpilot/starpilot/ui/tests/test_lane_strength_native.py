"""Native lane-strength presentation preserves canonical owner percent transport."""
from dataclasses import replace
import unittest
from unittest.mock import Mock, patch

from openpilot.starpilot.ui.feature_settings_state import (
  FeatureRow, lane_strength_level, row_change, row_default, value_reason, value_text,
)


class TestNativeLaneStrength(unittest.TestCase):
  def row(self, percent=100):
    return FeatureRow('LaneCenteringStrength', 'Lane Centering Strength', str(percent), b'1.5',
                      step=1, minimum=33, maximum=200, unit='%', available=True,
                      reason='100% is normal', default_value='100', vehicle_fingerprint='EXACT',
                      capability=('capability',), dependencies=(('LaneCentering', b'1'),))

  def test_labels_and_explicit_saved_values(self):
    for percent, label in ((33, '1'), (67, '3.03'), (100, '5'), (105, '5.25'), (150, '7.5'), (200, '10')):
      with self.subTest(percent=percent):
        row = self.row(percent)
        self.assertEqual(value_text(row), label)
        self.assertEqual(row.value, str(percent))
        self.assertEqual(row.source, b'1.5')
    self.assertIn('1–10; 5 is normal', value_reason(self.row()))
    blocked = replace(self.row(), available=False, reason='Vehicle unavailable')
    self.assertEqual(value_reason(blocked), 'Vehicle unavailable')
    self.assertIsNone(lane_strength_level(replace(self.row(), value='Invalid saved value', step=0)))
    self.assertEqual(value_text(replace(self.row(), key='OtherStrength')), '100%')

  def test_one_level_steps_preserve_owner_transport_and_guards(self):
    for percent, direction, expected in ((100, -1, '83'), (100, 1, '120'), (33, 1, '50'), (200, -1, '180')):
      row = self.row(percent)
      request = row_change(row, direction)
      self.assertIsNotNone(request)
      self.assertEqual(request.value, expected)
      self.assertEqual(request.expected, row.source)
      self.assertEqual(request.vehicle_fingerprint, row.vehicle_fingerprint)
      self.assertEqual(request.capability, row.capability)
      self.assertEqual(request.dependencies, row.dependencies)
    self.assertIsNone(row_change(self.row(33), -1))
    self.assertIsNone(row_change(self.row(200), 1))
    self.assertIsNone(row_change(replace(self.row(), available=False), 1))
    self.assertEqual(row_default(self.row()).value, '100')
    self.assertEqual(row_change(replace(self.row(), key='OtherStrength'), 1).value, '101.0')

  def test_actual_compact_buttons_display_and_dispatch(self):
    from openpilot.starpilot.ui import feature_settings_compact as compact
    class Button:
      def __init__(self, text, detail):
        self.text, self.detail, self.click = text, detail, None
      def set_click_callback(self, click):
        self.click = click
    sent = []
    session = Mock(spec=compact.FeatureSession, feature_request=lambda request: sent.append(request) or True)
    adapter = compact.FeatureSettingsCompact(session)
    refreshed = []
    with patch.object(compact, 'BigButton', Button):
      cards = adapter._editable(self.row(), lambda: refreshed.append(True), Mock(spec=compact.NavScroller))
      self.assertEqual(len(cards), 2)
      self.assertTrue(all(card.detail.startswith('5 · 1–10; 5 is normal') for card in cards))
      cards[1].click()
    self.assertEqual(sent[0].value, '120')
    self.assertEqual(sent[0].expected, b'1.5')
    self.assertEqual(refreshed, [True])
