#include <stdio.h>
#include <csignal>

void my_handler(int sig)
{
    printf("Caught signal %d\n", sig);
}

int main()
{

    std::signal(SIGINT, my_handler);

    pause();

    return 0;
}