from dataclasses import asdict,replace
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture,NOW
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots
from openpilot.starpilot.flm.torque_adapter import Admission
from openpilot.starpilot.flm.torque_raw_fit import control_from_dict


def test_actual_raw_point_child_keeps_typed_full30_trace():
  owner,p,v,context=fixture()
  owner.prepare(Admission(p,NOW,True,False,True,v),active=True,current_context=context)
  control,_,_=snapshots(NOW)
  control=replace(control,effect_trace=owner.trace,effect_prepare_ns=NOW)
  result=control_from_dict(asdict(control))
  assert result==control
  assert result.effect_trace.validate()==owner.trace
