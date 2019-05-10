/*!
    \file
    \brief system monitor (see \ref FF_MON)

    - Copyright (c) 2019 Philippe Kehl & flipflip industries (flipflip at oinkzwurgl dot org),
      https://oinkzwurgl.org/projaeggd/u-clox
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include "mon.h"

#define MON_OFFSET  1000
#define MAX_TASKS  25

#define NUMOF(x) (sizeof(x)/sizeof(*(x)))

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define TICKS2MS(ticks)           ((ticks) * portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));
#define osDelayUntil(prev, inc)   vTaskDelayUntil(prev, MS2TICKS(inc));

#define LOGNAME "mon"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)


static int sTaskSortFunc(const void *a, const void *b)
{
    //return (int)((const TaskStatus_t *)a)->xTaskNumber - (int)((const TaskStatus_t *)b)->xTaskNumber;

    const TaskStatus_t *pA = (const TaskStatus_t *)a;
    const TaskStatus_t *pB = (const TaskStatus_t *)b;
    return (pA->xCoreID != pB->xCoreID) ?
        ((int)pA->xCoreID - (int)pB->xCoreID) :
        ((int)pA->xTaskNumber - (int)pB->xTaskNumber);
}

static uint32_t sMonTick;
static uint32_t sMonPeriod;

void monSetPeriod(const int period)
{
    sMonTick = xTaskGetTickCount();
    if (period < 500)
    {
        sMonPeriod = 500;
    }
    else
    {
        sMonPeriod = MS2TICKS(period);
    }
    DEBUG("period=%u (%ums)", sMonPeriod, TICKS2MS(sMonPeriod));
}

static void sMonTask(void *pArg)
{
    //UNUSED(pArg);
    TaskStatus_t *pTasks = NULL;

    while (true)
    {
        if (pTasks != NULL)
        {
            free(pTasks);
            pTasks = NULL;
        }
        // wait until it's time to dump the status (0 = monitor off)
        if (sMonPeriod > 0)
        {
            static uint32_t sMonLastTick;
            vTaskDelayUntil(&sMonTick, MS2TICKS(100));
            //DEBUG("tick %u per %u rem %u", sMonTick, sMonPeriod, (sMonTick % sMonPeriod));
            if ( sMonTick >= (sMonLastTick + sMonPeriod) )
            {
                sMonLastTick = sMonTick;
            }
            else
            {
                continue;
            }
        }
        else
        {
            osSleep(42);
            continue;
        }

        const int nTasks = uxTaskGetNumberOfTasks();
        if (nTasks > MAX_TASKS)
        {
            ERROR("too many tasks");
            continue;
        }

        // allocate memory for tasks status
        const unsigned int allocSize = nTasks * sizeof(TaskStatus_t);
        pTasks = malloc(allocSize);
        if (pTasks == NULL)
        {
            ERROR("malloc");
            continue;
        }
        memset(pTasks, 0, allocSize);

        uint32_t totalRuntime;
        const int nnTasks = uxTaskGetSystemState(pTasks, nTasks, &totalRuntime);
        if (nTasks != nnTasks)
        {
            ERROR("%u != %u", nTasks, nnTasks);
            continue;
        }

        // sort by task ID
        qsort(pTasks, nTasks, sizeof(TaskStatus_t), sTaskSortFunc);

        // total runtime (tasks, OS, ISRs) since we checked last
        static uint32_t sLastTotalRuntime;
        {
            const uint32_t runtime = totalRuntime;
            totalRuntime = totalRuntime - sLastTotalRuntime;
            sLastTotalRuntime = runtime;
        }
        // calculate time spent in each task since we checked last
        static uint32_t sLastRuntimeCounter[MAX_TASKS];
        uint32_t totalRuntimeTasks = 0;
        for (int ix = 0; ix < nTasks; ix++)
        {
            TaskStatus_t *pTask = &pTasks[ix];
            const uint32_t runtime = pTask->ulRunTimeCounter;
            pTask->ulRunTimeCounter = pTask->ulRunTimeCounter - sLastRuntimeCounter[ix];
            sLastRuntimeCounter[ix] = runtime;
            totalRuntimeTasks += pTask->ulRunTimeCounter;
        }

        // FIXME: why?
        if (totalRuntimeTasks > totalRuntime)
        {
            totalRuntime = totalRuntimeTasks;
        }

        // print monitor info
        DEBUG("--------------------------------------------------------------------------------");
        DEBUG("sys: ticks=%u heap=%u/%u",
            sMonTick, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());

        // print tasks info
        for (int ix = 0; ix < nTasks; ix++)
        {
            const TaskStatus_t *pkTask = &pTasks[ix];
            char state = '?';
            switch (pkTask->eCurrentState)
            {
                case eRunning:   state = 'X'; break;
                case eReady:     state = 'R'; break;
                case eBlocked:   state = 'B'; break;
                case eSuspended: state = 'S'; break;
                case eDeleted:   state = 'D'; break;
                //case eInvalid:   state = 'I'; break;
            }
            const char core = pkTask->xCoreID == tskNO_AFFINITY ? '*' : ('0' + pkTask->xCoreID);

            char perc[8];
            if (pkTask->ulRunTimeCounter)
            {
                const double p = (double)pkTask->ulRunTimeCounter * 100.0 / (double)totalRuntimeTasks;
                if (p < 0.05)
                {
                    strcpy(perc, "<0.1%");
                }
                else
                {
                    snprintf(perc, sizeof(perc), "%5.1f%%", p);
                }
            }
            else
            {
                strcpy(perc, "0.0%");
            }
            DEBUG("tsk: %02d %-20s %c %c %2d-%2d %4u %6s",
                (int)pkTask->xTaskNumber, pkTask->pcTaskName, state, core,
                (int)pkTask->uxCurrentPriority, (int)pkTask->uxBasePriority,
                pkTask->usStackHighWaterMark, perc);
        }
        DEBUG("--------------------------------------------------------------------------------");
    }
}

void monStart(void)
{
    INFO("mon: start");

    monSetPeriod(5000);

    BaseType_t res = xTaskCreatePinnedToCore(
        sMonTask, LOGNAME, 4096 / sizeof(StackType_t), NULL, 20, NULL, 0);
    assert(res == pdPASS);
}

// eof
