void UnityAssertEqualInt(int expected, int actual, const char* message, int lineNumber);
void UnityAssertEqualString(const char* expected, const char* actual, const char* message, int lineNumber);
void UnityBegin(const char* filename);
void UnityEnd(void);
void UnityPrint(const char* message);
void UnityFail(const char* message, int lineNumber);