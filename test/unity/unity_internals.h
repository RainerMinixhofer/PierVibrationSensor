#ifndef UNITY_INTERNALS_H
#define UNITY_INTERNALS_H

#include "unity.h"

typedef struct {
    const char *testName;
    void (*testFunc)(void);
} UnityTest;

void UnityAddTest(const char *testName, void (*testFunc)(void));
void UnityRunTests(void);

#endif // UNITY_INTERNALS_H