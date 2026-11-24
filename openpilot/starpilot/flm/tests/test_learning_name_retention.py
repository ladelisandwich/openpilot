"""Actual Cereal named-field decoding keeps content and fixed cache bounds."""
import struct
import pytest
from openpilot.cereal import custom
from openpilot.starpilot.flm import learning_wire as wire


def row(values):
  return custom.LearningControlEvidence.new_message(replayInputs=values).as_reader().replayInputs


def test_actual_cereal_names_share_only_exact_validated_keys():
  wire._evidence_field_name.cache_clear()
  values=row([{'name':'steeringAngleDeg','number':-0.0},{'name':'steeringPressed','boolean':False},
              {'name':'sourceText','text':'unchanged'}])
  first=wire._read_named_values(values,32)
  second=wire._read_named_values(values,32)
  assert first==second
  assert all(a[0] is b[0] for a,b in zip(first,second,strict=True))
  assert struct.pack('!d',first[0][1])==struct.pack('!d',-0.0)
  assert type(first[1][1]) is bool and first[1][1] is False
  assert first[2][1]=='unchanged'
  different=wire._read_named_values(row([{'name':'steeringAngleDEG','number':1.}]),32)
  assert different[0][0]!=first[0][0]
  assert different[0][0] is not first[0][0]


@pytest.mark.parametrize('name',['','x'*97])
def test_invalid_names_never_enter_receiver_key_cache(name):
  wire._evidence_field_name.cache_clear()
  with pytest.raises(ValueError,match='bounded string'):
    wire._read_named_values(row([{'name':name,'number':1.}]),32)
  assert wire._evidence_field_name.cache_info().currsize==0


def test_receiver_key_cache_has_fixed_bound_and_original_wire_bound_remains():
  wire._evidence_field_name.cache_clear()
  for i in range(2200):
    value=wire._read_named_values(row([{'name':f'field-{i}','number':float(i)}]),32)
    assert value==((f'field-{i}',float(i)),)
  assert wire._evidence_field_name.cache_info().currsize==2048
  with pytest.raises(ValueError,match='replay scalar bounds'):
    wire._read_named_values(row([{'name':'field','number':1.}]*33),32)
  assert wire._evidence_field_name.cache_info().currsize==2048
