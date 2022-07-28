# coding: UTF-8
import time 
import signal
import rospy


def scheduler(arg1, arg2):
    print(1)

def main1(arg):
    for j in range(2):
        start = time.time()
        signal.signal(signal.SIGALRM, scheduler)
        signal.setitimer(signal.ITIMER_REAL, 0.1, 0.001)
        time.sleep(1)
        print(time.time() - start)

def intervalHandler(signum, frame):
    print("定期的に実行したい処理を書く")

def main2(arg):
    
    starttime = time.time()
    
    # インターバルを起床するハンドラを設定(SIGALRMハンドラー)
    signal.signal(signal.SIGALRM, intervalHandler)

    # インターバルタイマー設定
    # 最初の開始タイミング１秒後。以降入り一秒間隔
    signal.setitimer(signal.ITIMER_REAL, 1, 1)

    try:
        while True:

        # インターバルタイマーの待ち時間はずっとsleep
        # 時間は取り扱えず何でもいいが電力消費抑えるなら長めで
            time.sleep(100)
            
            if time.time() - starttime > 3:
                
                break
        
    except KeyboardInterrupt:
        print('_nCTRL-C pressed!')

        sys.exit()
    time.sleep(2)
    time.sleep(3)
    print('END')

if __name__ == "__main__":
    main2("start")