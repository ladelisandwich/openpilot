#define _GNU_SOURCE
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <unistd.h>
#include "../../native/audio_mixer.c"
static struct audio *engine;
static pthread_mutex_t barrier_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t barrier_cond = PTHREAD_COND_INITIALIZER;
static unsigned barrier_arrived;
static void barrier_wait(void) {
  assert(pthread_mutex_lock(&barrier_mutex) == 0);
  barrier_arrived++;
  if (barrier_arrived == 2) assert(pthread_cond_broadcast(&barrier_cond) == 0);
  while (barrier_arrived < 2) assert(pthread_cond_wait(&barrier_cond, &barrier_mutex) == 0);
  assert(pthread_mutex_unlock(&barrier_mutex) == 0);
}
static _Atomic int done;
static uint64_t reader_attempts,reader_successes,reader_unavailable,accepted_unfrozen,accepted_wrapped_unfrozen;
static void validate(const uint64_t *out,unsigned words) {
  assert(out[0]==1 && out[1]==22 && out[2]==32 && out[7]<=32);
  assert(words==10+out[7]*22);
  for(uint64_t n=0;n<out[7];n++) {
    const uint64_t *row=out+10+n*22;
    assert(row[0]==out[6]+n && row[8]==64 && row[18]==0 && row[19]==1);
    assert(row[3]>=row[1] && row[4]>=row[2]);
    assert(row[10]==2 && row[11]==1 && row[12]==0 && row[14]==1);
  }
  if(out[5])assert(out[4] && out[3]==out[4]+4 && out[7]==21);
}
static void *reader(void *unused) {
  (void)unused;uint64_t out[714];
  barrier_wait();
  while(!atomic_load(&done)) {
    unsigned words=audio_diagnostic_snapshot(engine,out,714);reader_attempts++;
    if(words) {validate(out,words);reader_successes++;if(!out[5]){accepted_unfrozen++;if(out[3]>32)accepted_wrapped_unfrozen++;}}else reader_unavailable++;
  }
  unsigned words=audio_diagnostic_snapshot(engine,out,714);assert(words);validate(out,words);
  return NULL;
}
int main(void) {
  alarm(15);engine=audio_create();assert(engine);
  float pcm[1024];for(unsigned i=0;i<1024;i++)pcm[i]=(float)(i%31)/31.f;
  const float *p[]={pcm};unsigned lengths[]={1024};float output[64];struct pa_time t={0,1,2};
  assert(audio_publish_bank(engine,p,lengths,1,1));assert(audio_command(engine,1,0,-1,0,.25f,1,1,0)==2);assert(audio_prepare(engine));
  pthread_t thread;assert(pthread_create(&thread,NULL,reader,NULL)==0);
  barrier_wait();
  for(unsigned n=0;n<100000;n++){assert(audio_callback(NULL,output,64,&t,n==99990?4:0,engine)==0);if(n%128==0)sched_yield();}
  // Test-only additional postfreeze yields; prefreeze writer also yields every128 calls.
  for(unsigned n=0;n<1000;n++){assert(audio_callback(NULL,output,64,&t,0,engine)==0);sched_yield();}
  atomic_store(&done,1);assert(pthread_join(thread,NULL)==0);assert(reader_attempts>0 && reader_successes>0 && accepted_unfrozen>0 && accepted_wrapped_unfrozen>0);
  uint64_t out[714];assert(audio_diagnostic_snapshot(engine,out,714)==472);assert(out[3]==99995 && out[4]==99991 && out[5]==1);
  assert(engine->underflows==1 && atomic_load(&engine->diag_abort_count)==0 && atomic_load(&engine->diag_clock_errors)==0);
  printf("{\"attempts\":%llu,\"accepted\":%llu,\"boundedUnavailable\":%llu,\"joined\":true}\n",(unsigned long long)reader_attempts,(unsigned long long)reader_successes,(unsigned long long)reader_unavailable);
  printf("{\"acceptedUnfrozen\":%llu,\"acceptedWrappedUnfrozen\":%llu}\n",(unsigned long long)accepted_unfrozen,(unsigned long long)accepted_wrapped_unfrozen);
  assert(pthread_cond_destroy(&barrier_cond)==0);assert(pthread_mutex_destroy(&barrier_mutex)==0);audio_destroy(engine);alarm(0);return 0;
}
