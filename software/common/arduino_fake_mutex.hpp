#ifndef FAKE_MUTEX_H
#define FAKE_MUTEX_H


/* So this one serves a strange purpose:
   When compiled for simulation, it wraps he std library Mutex and is 
   real and threadsafe.
   When compiled for Arduino, it implements a NON THREAD SAFE mutex.
   Useless? Kinda. If you can work out how to build a thread-safe mutex on
   Arduino with the Thumb instruction set, let me know */

#ifdef COMPILE_FOR_ARDUINO

class Mutex { 
  private: 
    volatile int lock_var = 0; // actual lock global variable used to provide synchronization

  public:
    bool try_lock()
    {
        volatile int ret = lock_var;
        lock_var = 1;
        return ret == 0;
    }

    void lock()
    {
        bool have_lock = false;
        while (!have_lock)
            have_lock = try_lock();
    }

    void unlock()
    {
        lock_var = 0;
    }
};

#else

#include <pthread.h>
class Mutex { 
  private: 
    pthread_mutex_t mutex;

  public:
    Mutex(){
      pthread_mutex_init(&mutex, NULL);
    }

    bool try_lock()
    {
        return pthread_mutex_trylock(&mutex);
    }

    void lock()
    {
        pthread_mutex_lock(&mutex);
    }

    void unlock()
    {
        pthread_mutex_unlock(&mutex);
    }
};
#endif

#endif