#define _GNU_SOURCE
#include <time.h>
#include <assert.h>
#include <stdint.h>
static unsigned clock_calls;
static int fail_clocks;
static int fake_clock(clockid_t id,struct timespec *ts) {
  (void)id;clock_calls++;
  if(fail_clocks)return -1;
  ts->tv_sec=1;ts->tv_nsec=clock_calls*1000;return 0;
}
#define clock_gettime fake_clock
#include "../../native/audio_mixer.c"
#undef clock_gettime
static struct audio *fixture(void) {
  struct audio *a=audio_create();assert(a);
  static const float pcm[]={.25f,.5f,.75f,1.f};
  const float *p[]={pcm};const unsigned lengths[]={4};
  assert(audio_publish_bank(a,p,lengths,1,1));
  assert(audio_command(a,1,0,-1,0,0.f,1,1,0)==2);
  assert(audio_prepare(a));return a;
}
int main(void) {
  uint64_t out[DIAG_HEADER+DIAG_ROWS*DIAG_WORDS];float rendered[4];
  struct pa_time t={1.,2.,3.};struct audio *a=fixture();
  for(unsigned i=0;i<20;i++)assert(audio_callback(NULL,rendered,4,&t,0,a)==0);
  assert(audio_callback(NULL,rendered,4,&t,4,a)==0);
  for(unsigned i=0;i<4;i++)assert(audio_callback(NULL,rendered,4,&t,0,a)==0);
  assert(clock_calls==100);assert(audio_diagnostic_snapshot(a,out,714)==10+21*22);
  assert(out[0]==1 && out[3]==25 && out[4]==21 && out[5]==1 && out[6]==5 && out[7]==21);
  for(unsigned i=0;i<21;i++)assert(out[10+i*22]==5+i);
  assert(out[10+16*22+9]==4);assert(out[10+16*22+16]==1);
  for(unsigned i=0;i<4;i++)assert(rendered[i]==0.f); // Gain0 still advances real cursor.
  assert(a->frame==100);assert(a->underflows==1);
  for(unsigned i=0;i<3;i++)assert(audio_callback(NULL,rendered,4,&t,0,a)==0);
  assert(clock_calls==100 && a->frame==112); // Frozen ring adds no further clock calls.
  assert(audio_diagnostic_snapshot(a,out,713)==0);
  atomic_store(&a->diag_seq,3);assert(audio_diagnostic_snapshot(a,out,714)==0);atomic_store(&a->diag_seq,4);
  atomic_flag_test_and_set(&a->rendering);assert(audio_callback(NULL,rendered,4,&t,0,a)==2);
  assert(atomic_load(&a->diag_abort_count)==1);atomic_flag_clear(&a->rendering);audio_destroy(a);
  a=fixture();for(unsigned i=0;i<40;i++)assert(audio_callback(NULL,rendered,4,NULL,0,a)==0);
  assert(audio_diagnostic_snapshot(a,out,714)==714);assert(out[3]==40 && out[4]==0 && out[5]==0 && out[6]==9 && out[7]==32);
  assert(out[10+19]==0); // Missing PaTimeInfo explicit.
  fail_clocks=1;assert(audio_callback(NULL,rendered,4,NULL,0,a)==0);
  assert(audio_diagnostic_snapshot(a,out,714)==714);assert(out[9]==1);
  unsigned last=10+31*22;assert(out[last+18]==15 && out[last+1]==0 && out[last+4]==0);
  audio_destroy(a);return 0;
}
