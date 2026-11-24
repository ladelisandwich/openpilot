"""EPS73 retrofit filter ownership preserves both existing identities and tunes."""
import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import prius_retrofit_advert as owner, prius_advert_filter, prius_longitudinal
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota import aol


def firmware():
  return [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x7A1,
                                fwVersion=owner.EPS_FIRMWARE, brand='toyota', bus=0)]


def factory(candidate, *, length=4, zss=False, alpha=False, fp=None, fw=None):
  if fp is None:
    fp=gen_empty_fingerprint()
    fp[0][0x2FF]=length
    if zss:
      fp[0][0x23]=8
  return CarInterface.get_params(candidate, fp, firmware() if fw is None else fw, alpha, False, False)


@pytest.mark.parametrize('candidate', owner.CARS)
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('zss', (False, True))
def test_eps73_fourbyte_factory_preserves_existing_retrofit_tunes(candidate, stock, alpha, zss):
  cp=factory(candidate, alpha=alpha, zss=zss)
  before=factory(candidate, length=8, alpha=alpha, zss=zss)
  assert not cp.dashcamOnly and owner.configured(cp)
  assert cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam==owner.LONG_WORD
  if stock:
    assert prius_longitudinal.prepare_stock(cp) and prius_longitudinal.prepare_stock(before)
  assert cp.safetyConfigs[0].safetyParam==(owner.STOCK_WORD if stock else owner.LONG_WORD)
  assert cp.openpilotLongitudinalControl==(not stock)
  assert prius_advert_filter.configured(cp) and aol.qualified(cp)
  actual,baseline=cp.to_dict(),before.to_dict()
  actual.pop('safetyConfigs')
  baseline.pop('safetyConfigs')
  assert actual==baseline, 'Existing EPS73 config and all tunes must be preserved'


@pytest.mark.parametrize('candidate', owner.CARS)
@pytest.mark.parametrize('bad', ('primary7','mirror8','bus1','pedal0','pedal2','twoaa0','twoaa2','eps5','main7','camera7'))
def test_eps73_fourbyte_rejects_ambiguous_topology(candidate,bad):
  fp=gen_empty_fingerprint()
  fp[0][0x2FF]=4
  changes={'primary7':(0,0x2FF,7),'mirror8':(2,0x2FF,8),'bus1':(1,0x2FF,4),
           'pedal0':(0,0x201,5),'pedal2':(2,0x201,6),'twoaa0':(0,0x2AA,8),'twoaa2':(2,0x2AA,8),
           'eps5':(0,0x262,5),'main7':(0,0x1D3,7),'camera7':(2,0x343,7)}
  bus,address,size=changes[bad]
  fp[bus][address]=size
  assert not owner.hardware_supported(candidate,fp,firmware())
  cp=factory(candidate,fp=fp)
  assert not owner.configured(cp) and cp.safetyConfigs[0].safetyParam not in owner.WORDS


@pytest.mark.parametrize('candidate', owner.CARS)
@pytest.mark.parametrize('bad', ('logging','bus','brand','address','subAddress','fwVersion','missing'))
def test_eps73_fourbyte_requires_actual_eps_identity(candidate,bad):
  fw=firmware()
  if bad=='missing':
    fw=[]
  else:
    values={'logging':True,'bus':1,'brand':'gm','address':0x7A2,'subAddress':1,'fwVersion':b'unknown'}
    setattr(fw[0],bad,values[bad])
  fp=gen_empty_fingerprint()
  fp[0][0x2FF]=4
  assert not owner.hardware_supported(candidate,fp,fw)
  assert not owner.configured(factory(candidate,fp=fp,fw=fw))


@pytest.mark.parametrize('candidate', owner.CARS)
@pytest.mark.parametrize('field,value', (('notCar',True),('passive',True),('dashcamOnly',True),('secOcRequired',True),
                                      ('pcmCruise',False),('alphaLongitudinalAvailable',True),('autoResumeSng',True),
                                      ('brand','gm'),('steerControlType',structs.CarParams.SteerControlType.angle),
                                      ('flags',0),('flags',1<<30),('carFingerprint',CAR.TOYOTA_RAV4H)))
def test_eps73_fourbyte_refuses_foreign_configuration(candidate,field,value):
  cp=factory(candidate)
  setattr(cp,field,value)
  assert not owner.configured(cp) and not prius_advert_filter.configured(cp)


@pytest.mark.parametrize('candidate', owner.CARS)
@pytest.mark.parametrize('word', (0xC042,0xC242,0xC049,4169,4681))
def test_eps73_fourbyte_does_not_reuse_another_native_identity(candidate,word):
  cp=factory(candidate)
  cp.safetyConfigs[0].safetyParam=word
  assert not owner.configured(cp)
