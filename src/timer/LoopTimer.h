//LoopTimer.h

#ifndef SAI_LOOPTIMER_H_
#define SAI_LOOPTIMER_H_

#include <string>
#include <iostream>

#include <unistd.h>
// #include <sys/resource.h>
// #include <sched.h>
// #include <sys/mman.h>
#include <signal.h>
#include <time.h>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

/** \brief Accurately time a loop to set frequency.
 *
 */
class LoopTimer {

public:

    LoopTimer(){}
    
    virtual ~LoopTimer(){}

    /** \brief Set the loop frequency
     * \param frequency The loop frequency that will be used for LoopTimer::run()
     */
    void setLoopFrequency(double frequency){
        update_interval_ = 1000000000.0/frequency;
    }

    /** \brief Initialize the timing loop, if using your own while loop. call before waitForLoop.
     * \param initial_wait_nanoseconds The delay before waitForNextLoop will return the first time
     */
    void initializeTimer(unsigned int initial_wait_nanoseconds = 0)
    {   
        // initialize time
        getCurrentTime(t_next_);
        
        // calculate next shot. carry over nanoseconds into seconds.
        t_next_.tv_nsec += initial_wait_nanoseconds;
        while (t_next_.tv_nsec >= 1000000000){
            t_next_.tv_nsec -= 1000000000;
            t_next_.tv_sec++;
        }
        t_start_ = t_next_;
        // TODO os x
        // http://stackoverflow.com/questions/11338899/are-there-any-well-behaved-posix-interval-timers
    }

    /** \brief Wait for next loop. Use in your while loop. Not needed if using LoopTimer::run().
     * \return true if a wait was required, and false if no wait was required. */
    bool waitForNextLoop()
    {
        // grab the time
        getCurrentTime(t_curr_);

        // wait until next shot
        nanoSleepUntil(t_next_);

        // check if it slept
        bool slept = true;
        while (t_curr_.tv_nsec >= 1000000000){
            t_curr_.tv_nsec -= 1000000000;
            t_curr_.tv_sec++;
        }
        if (t_curr_.tv_sec >= t_next_.tv_sec && t_curr_.tv_nsec > t_next_.tv_nsec){
            slept = false;
        }

        // calculate dt
        t_loop_ = elapsedTime(t_start_, t_curr_);

        // calculate next shot. carry over nanoseconds into seconds.
        t_next_.tv_nsec += update_interval_;
        while (t_next_.tv_nsec >= 1000000000){
            t_next_.tv_nsec -= 1000000000;
            t_next_.tv_sec++;
        }
        ++update_counter_;

        return slept;
    }

    /** \brief Time when waitForNextLoop was last called */
    void loopTime(timespec& t)
    {
        t = t_loop_;
    }

    /** \brief Time when waitForNextLoop was last called */
    double loopTime()
    {
        return t_loop_.tv_sec + 0.000000001*t_loop_.tv_nsec;
    }

    /** \brief Run a loop that calls the user_callback(). Blocking function.
     * \param userCallback A function to call every loop.
     */
    void run( void (*userCallback)(void) ){
        initializeTimer(update_interval_);

        running_ = true;
        while(running_)
        {
            waitForNextLoop();
            userCallback();
        }
    }

    /** \brief Stop the loop, started by run(). Use within callback, or from a seperate thread. */
    void stop(){   
        running_ = false;
    }

    /** \brief Add a ctr-c exit callback.
     * \param userCallback A function to call when the user presses ctrl-c.
     */
    static void setCtrlCHandler( void (*userCallback)(int) ){
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = userCallback;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
    }

    /** \brief Elapsed time since calling initializeTimer() or run() in seconds. */
    void elapsedTime(timespec& t){
        struct timespec t_now;
        getCurrentTime(t_now);
        t = elapsedTime(t_start_,t_now);
    }

    /** \brief Elapsed time since calling initializeTimer() or run() in seconds. */
    double elapsedTime(){
        struct timespec t;
        elapsedTime(t);
        return t.tv_sec + 0.000000001*t.tv_nsec;
    }

    /** \brief Number of loops since calling run. */
    unsigned long long elapsedCycles(){
        return update_counter_;
    }

    // /** \brief Set the thread to a priority of -19. Priority range is -20 (highest) to 19 (lowest) */
    // static void setThreadHighPriority(){
    //     pid_t pid = getpid();
    //     int priority_status = setpriority(PRIO_PROCESS, pid, -19);
    //     if (priority_status){
    //         printWarning("setThreadHighPriority. Failed to set priority.");
    //     }
    // }

    // /** \brief Set the thread to real time (FIFO). Thread cannot be preempted. 
    //  *  Set priority as 49 (kernel and interrupts are 50).
    //  * \param MAX_SAFE_STACK maximum stack size in bytes which is guaranteed safe to access without faulting
    //  */
    // static void setThreadRealTime(const int MAX_SAFE_STACK = 8*1024)
    // {
    //     // Declare ourself as a real time task, priority 49.
    //     // PRREMPT_RT uses priority 50
    //     // for kernel tasklets and interrupt handler by default
    //     struct sched_param param;
    //     param.sched_priority = 49;
    //    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //         perror("sched_setscheduler failed");
    //         exit(-1);
    //     }

    //     // Lock memory
    //     if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    //         perror("mlockall failed");
    //         exit(-2);
    //     }

    //     // Pre-fault our stack
    //     //int MAX_SAFE_STACK = 8*1024;
    //     unsigned char dummy[MAX_SAFE_STACK];
    //     memset(dummy, 0, MAX_SAFE_STACK);
    // }

protected:

    inline timespec elapsedTime(timespec start, timespec end)
    {
        timespec dt;
        if ((end.tv_nsec-start.tv_nsec)<0) {
            dt.tv_sec = end.tv_sec-start.tv_sec-1;
            dt.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        } else {
            dt.tv_sec = end.tv_sec-start.tv_sec;
            dt.tv_nsec = end.tv_nsec-start.tv_nsec;
        }
        return dt;
    }

    inline void getCurrentTime(timespec &t_ret) {
#ifdef __APPLE__
        static double ratio = 0.0;
        if (!ratio) {
            mach_timebase_info_data_t info;
            if (mach_timebase_info(&info) == KERN_SUCCESS) {
                ratio = info.numer * info.denom;
            }
        }
        uint64_t t_nsecs = mach_absolute_time() * ratio;
        t_ret.tv_sec = 0;
        while (t_nsecs >= 1000000000){
            t_nsecs -= 1000000000;
            t_ret.tv_sec++;
        }
        t_ret.tv_nsec = static_cast<long>(t_nsecs);
#else
        clock_gettime(CLOCK_MONOTONIC, &t_ret);
#endif
    }

    inline void nanoSleepUntil(const timespec &t_next) {
#ifdef __APPLE__
        timespec t_now, t_sleep;
        getCurrentTime(t_now);
        t_sleep = elapsedTime(t_now, t_next_);
        nanosleep(&t_sleep, NULL);
#else
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next_, NULL);
#endif
    }

    static void printWarning(const std::string& message){
        std::cout << "WARNING. LoopTimer. " << message << std::endl;
    }

    volatile bool running_ = false;

    struct timespec t_next_;
    struct timespec t_curr_;
    struct timespec t_start_;
    struct timespec t_loop_;

    unsigned long long update_counter_ = 0;

    unsigned int update_interval_ = 1000000000.0/1000; // 1000 Hz
    
};


#endif /* SAI_LOOPTIMER_H_ */
