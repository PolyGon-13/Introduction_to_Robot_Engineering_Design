VERBOSE = True일 때 출력되는 내용
1. 시리얼 포트가 열렸을 때
[OK] Serial port opened: /dev/ttyS0 @ 9600
2. 프로그램 시작 안내문
=== Project 1 : Straight Drive Controller ===
 enter distance(m)        e.g.  1.5       (forward 1.5 m)
                                -0.3      (backward 0.3 m)
 's'                      emergency stop
 'q'                      quit
3. 사용자 입력 프롬프트
>>
4. 라즈베리파이가 아두이노로 명령을 보낼 때
[SEND] D 1.5000

또는

[SEND] S
5. 아두이노가 시리얼로 메시지를 보낼 때
[ARDUINO] ...

예시:

[ARDUINO] DONE 1012 1008 1.0000 0.9960 0.9980
6. 아두이노의 DONE 메시지를 상세 파싱했을 때
---------------------------------------------
  enc_r = 1012
  enc_l = 1008
  d_r   = 1.0000 m
  d_l   = 0.9960 m
  d_avg = 0.9980 m
  diff  = +4.0 mm  (R - L)
---------------------------------------------
7. 잘못된 입력을 넣었을 때
[ERROR] invalid number. enter meters (e.g. 1.5) or 's'/'q'
8. 입력한 거리값이 너무 작을 때
[ERROR] distance too small
9. DONE 응답이 제한 시간 안에 오지 않았을 때
[WARN] DONE not received within timeout. sending STOP for safety.

이후 자동으로 정지 명령을 보낸다.

[SEND] S
10. Ctrl+C를 눌렀을 때
[CTRL-C] sending STOP

이후 자동으로 정지 명령을 보낸다.

[SEND] S
11. 시리얼 포트를 열지 못했을 때
[ERROR] couldn't open serial: ...