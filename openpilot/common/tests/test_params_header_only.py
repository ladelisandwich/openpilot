"""Keys declared in params_keys.h that the compiled params library does not know are served from <params>/dx
(common/params.py): the same get/put surface, kept across a clear_all, never shadowing a compiled key."""
import os
import tempfile
from unittest import mock

from openpilot.common.test import OpenpilotTestCase
from openpilot.common import params as params_module
from openpilot.common.params import ParamKeyType, Params, UnknownKeyName, _HEADER_ONLY_KEYS, _HeaderOnlyStore

# the device's prebuilt library predates these keys; a library built here from the same header knows them, so the
# device is simulated by having the library reject them
FORK_KEYS = frozenset(("TorqueInterceptorEnabled", "RadarInterceptorEnabled", "RadarEmulationEnabled", "LowerMinSetSpeed",
                       "MazdaHybridLong", "MazdaSteerLimitClassifier", "MazdaTcsOffAlert", "MazdaWiperWeatherMode",
                       "MazdaHornBookmark", "MazdaHybridMaster", "NoMRCC", "NoFSC", "BlendedACC", "ManualTransmission",
                       "EcuDisableFailed", "TetheringEnabled"))


class TestHeaderOnlyParams(OpenpilotTestCase):
  def setUp(self):
    self.tmpdir = tempfile.mkdtemp(prefix="params_header_only_")
    real_check = params_module.params_check_key
    patcher = mock.patch.object(params_module, "params_check_key",
                                lambda handle, key: key.decode() not in FORK_KEYS and real_check(handle, key))
    patcher.start()
    self.addCleanup(patcher.stop)
    self.params = Params(self.tmpdir)
    # a header-only key is one the header declares and the library rejects
    self.header_only = [k for k in _HEADER_ONLY_KEYS if self.params._header_only(k) is not None]

  def test_fork_keys_are_header_only_here(self):
    for key in ("TorqueInterceptorEnabled", "RadarEmulationEnabled", "MazdaHybridLong", "MazdaHybridMaster", "TetheringEnabled"):
      self.assertIn(key, self.header_only, key)
    # a compiled key is never served from the file store, whatever the header says
    self.assertIsNone(self.params._header_only("IsMetric"))

  def test_bool_round_trip_and_defaults(self):
    self.assertTrue(self.params.get_bool("TorqueInterceptorEnabled"))      # declared default "1"
    self.assertFalse(self.params.get_bool("RadarEmulationEnabled"))        # no default: off
    self.params.put_bool("RadarEmulationEnabled", True)
    self.assertTrue(self.params.get_bool("RadarEmulationEnabled"))
    self.assertTrue(self.params.get("RadarEmulationEnabled"))
    self.assertEqual(self.params.get_type("RadarEmulationEnabled"), ParamKeyType.BOOL)
    self.params.remove("RadarEmulationEnabled")
    self.assertFalse(self.params.get_bool("RadarEmulationEnabled"))
    self.assertIsNone(self.params.get("RadarEmulationEnabled"))
    self.assertEqual(self.params.get("RadarEmulationEnabled", return_default=True), None)
    self.assertTrue(self.params.get("TorqueInterceptorEnabled", return_default=True))

  def test_int_round_trip_and_the_convenience_names(self):
    self.assertEqual(self.params.get_int("MazdaHybridMaster"), 0)
    self.params.put_int("MazdaHybridMaster", 5)
    self.assertEqual(self.params.get_int("MazdaHybridMaster"), 5)
    self.assertEqual(self.params.get("MazdaHybridMaster"), 5)
    self.params.put("TetheringEnabled", 2)
    self.assertEqual(self.params.get_int("TetheringEnabled", default=0), 2)
    self.assertEqual(self.params.get_default_value("TetheringEnabled"), 0)
    self.params.put_bool_nonblocking("MazdaHornBookmark", True)
    self.assertTrue(self.params.get_bool("MazdaHornBookmark"))

  def test_files_live_beside_the_library_store_and_survive_clear_all(self):
    self.params.put_bool("MazdaHybridLong", True)
    store = _HeaderOnlyStore(self.params.get_param_path())
    self.assertTrue(os.path.isfile(store.path("MazdaHybridLong")))
    self.assertEqual(self.params.get_param_path("MazdaHybridLong"), store.path("MazdaHybridLong"))
    self.params.clear_all()
    self.assertTrue(self.params.get_bool("MazdaHybridLong"))
    self.assertIn(b"MazdaHybridLong", self.params.all_keys())

  def test_unknown_keys_still_raise(self):
    with self.assertRaises(UnknownKeyName):
      self.params.get_bool("NoSuchKeyAnywhere")
    with self.assertRaises(UnknownKeyName):
      self.params.put_bool("NoSuchKeyAnywhere", True)
