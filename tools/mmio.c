#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

int main(int argc, char **argv)
{
    int fd = open(argv[1], O_RDWR | O_SYNC);
    volatile uint32_t *r = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, 0);

    /* 인자로 오프셋과 값을 받아 쓰고 되읽기 */
    if (argc >= 4) {
        unsigned off = strtoul(argv[2], NULL, 0);
        uint32_t v   = strtoul(argv[3], NULL, 0);
        r[off / 4] = v;
        printf("wrote 0x%08x to 0x%04x, read back 0x%08x\n",
            v, off, r[off / 4]);
    }

    // for (int off = 0; off <= 0x44; off += 4)
    //     printf("0x%04x: 0x%08x\n", off, r[off / 4]);

    return 0;
}