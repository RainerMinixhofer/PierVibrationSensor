void setUp(void) {
}

void tearDown(void) {
}

void test_hello(void) {
    // Example test case
    TEST_ASSERT_EQUAL(1, 1);
}

int main(void) {
    UnityBegin("test/test_hello/test_hello.c");
    RUN_TEST(test_hello);
    return UnityEnd();
}