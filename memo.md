doorbell register
doorbell register에 cpu가 값을 쓰면 controller가 명령을 실행.
doorbell register에 값을 쓴다는 것은 실행할 수 있는 명령어가 있다는 것을 controller에게 알리는 것이다.
dma_wmb를 통해서 이전에 쓴 결과가 바로 host에게 보이는 것을 강제할 수 있다.

read modify write : 메모리에서 값을 읽고 수정하고 쓴다. 동시성 문제가 발생할 수 있다.
