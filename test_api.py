"""quick test: stream accel + gyro from macimu"""
import time, sys
from macimu import IMU

RST = "\033[0m"; DIM = "\033[2m"; CYN = "\033[96m"; YEL = "\033[93m"
BWHT = "\033[97m"; BOLD = "\033[1m"
HIDE = "\033[?25l"; SHOW = "\033[?25h"; CLR = "\033[2J\033[H"

def bar(v, lo, hi, w=30):
    t = max(0.0, min(1.0, (v - lo) / (hi - lo)))
    pos = int(t * (w - 1))
    mid = int((0.0 - lo) / (hi - lo) * (w - 1))
    b = list('─' * w)
    if 0 <= mid < w: b[mid] = '┼'
    b[max(0, min(w - 1, pos))] = '●'
    return ''.join(b)

def main():
    with IMU() as imu:
        sys.stdout.write(HIDE + CLR)
        n = 0
        try:
            while True:
                time.sleep(0.05)
                accel = imu.read_accel()
                gyro = imu.read_gyro()
                if not accel:
                    continue
                n += len(accel)
                ax, ay, az = accel[-1]
                gx, gy, gz = gyro[-1] if gyro else (0, 0, 0)
                L = []
                L.append(f"{DIM}┌─{BWHT}{BOLD} macimu test {RST}{DIM}{'─' * 42}┐{RST}")
                L.append(f"{DIM}│{RST}  {DIM}samples: {n:,}{' ' * 44}{DIM}│{RST}")
                L.append(f"{DIM}├─ accelerometer (g) ─{'─' * 34}┤{RST}")
                L.append(f"{DIM}│{RST}  {CYN}X{RST} {bar(ax,-2,2)} {BWHT}{ax:>+9.5f}{RST} g   {DIM}│{RST}")
                L.append(f"{DIM}│{RST}  {CYN}Y{RST} {bar(ay,-2,2)} {BWHT}{ay:>+9.5f}{RST} g   {DIM}│{RST}")
                L.append(f"{DIM}│{RST}  {CYN}Z{RST} {bar(az,-2,2)} {BWHT}{az:>+9.5f}{RST} g   {DIM}│{RST}")
                L.append(f"{DIM}├─ gyroscope (°/s) ──{'─' * 35}┤{RST}")
                L.append(f"{DIM}│{RST}  {YEL}X{RST} {bar(gx,-250,250)} {BWHT}{gx:>+9.2f}{RST} °/s {DIM}│{RST}")
                L.append(f"{DIM}│{RST}  {YEL}Y{RST} {bar(gy,-250,250)} {BWHT}{gy:>+9.2f}{RST} °/s {DIM}│{RST}")
                L.append(f"{DIM}│{RST}  {YEL}Z{RST} {bar(gz,-250,250)} {BWHT}{gz:>+9.2f}{RST} °/s {DIM}│{RST}")
                L.append(f"{DIM}└{'─' * 55}┘{RST}")
                L.append(f"  {DIM}ctrl+c to quit{RST}")
                sys.stdout.write(CLR + '\n'.join(L))
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            sys.stdout.write(SHOW + '\n')

if __name__ == '__main__':
    main()
