/*
    励磁パターン
    一層励磁 二相励磁 1-2相励磁
        0001    0011      0001
        0010    0110      0011
        0100    1100      0010
        1000    1001      0110
                          0100
                          1100
                          1000
                          1001

    励磁パターンの選択(excitation()メソッド)

    1-2相励磁の時
        ステップ数が偶数のときは一相励磁、奇数のときは二相励磁のパターンを使う
    その他の励磁方式の時
        列挙型のインデックスをパターンのインデックスとして使う

    PATTERNS変数
        下位16ビット->一相励磁のパターン
        上位16ビット->二相励磁のパターン
        一相励磁のパターンを使うときは右に0ビットシフト
        二相励磁のパターンを使うときは右に16ビットシフト
        |1001|1100|0110|0011|1000|0100|0010|0001|
                            ^                   ^
                           二相                一相
                           1*16                0*16

        コイルのパターンの取得
            1.現在のステップを4で割った余り(0~3のパターン)に変換(1-2相励磁の場合同じインデックスを2回使うため割る2をする)
                0001 (一相励磁の一個目)
                0011 (二相励磁の一個目)
                ...
            2.一つのコイルパターンは4ビット分のため、4を掛けて先頭ビットの位置を計算する
                例:一相励磁の場合
                    |1000|0100|0010|0001|
                         ^    ^    ^    ^
                         3    2    1    0
                        3*4  2*4  1*4  0*4
            3.その数だけ右にビットシフトし、対象のパターンの先頭ビットをLSBに置く
            4.取得したビットパターンの下位4ビットを使用する
*/

#include "ULN2003.h"

/* public */

ULN2003::ULN2003(const uint8_t pinA, const uint8_t pinB, const uint8_t pinC, const uint8_t pinD,
                 const Phase phase, const uint16_t res, uint32_t delayCorrection) :
    PINS{ pinA, pinB, pinC, pinD }, resolution_(res), startSpeed_(10), currentStep_(0), toStep_(0), interval_(0), intervalAdj_(0),
    delayCorrection_(delayCorrection), acceleration_(0), currentInterval_(0), decreaseStep_(0),
    isBlocking_(false), isInfinite_(false), state_(State::STOPPED), phase_(phase), dir_(Dir::CW), interrupter_(micros) {
    setRPM(10);

    for (uint8_t pin : PINS) {
        pinMode(pin, OUTPUT);
    }
}

uint8_t ULN2003::getStartSpeed() const { return startSpeed_; }
void ULN2003::setStartSpeed(const uint8_t speed) { startSpeed_ = speed; }

uint32_t ULN2003::getRPM() const {
    if (interval_ == 0) { return 0; }
    return calcSpeed(interval_);
}
void ULN2003::setRPM(const uint32_t rpm) {
    if ((state_ != State::STOPPED) || (rpm == 0)) { return; }
    interval_ = calcSpeed(rpm);
}

uint32_t ULN2003::getInterval() const { return interval_; }
void ULN2003::setInterval(const uint32_t interval) {
    if (state_ != State::STOPPED) { return; }
    interval_ = interval;
}

ULN2003::Phase ULN2003::getPhase() const { return phase_; }
void ULN2003::setPhase(const Phase phase) {
    if (state_ != State::STOPPED) { return; }
    phase_ = phase;
}

uint32_t ULN2003::getDelayCorrection() const { return delayCorrection_; }
void ULN2003::setDelayCorrection(const uint32_t delayCorrection) {
    delayCorrection_ = delayCorrection;
    intervalAdj_ = (interval_ > delayCorrection_) ? interval_ - delayCorrection_ : 1;
}

uint16_t ULN2003::getAcceleration() const { return acceleration_; }

void ULN2003::setAcceleration(const uint16_t a) {
    if (state_ != State::STOPPED) { return; }
    acceleration_ = a;
}

bool ULN2003::isBlocking() const { return isBlocking_; }
void ULN2003::setBlocking(const bool isBlocking) { isBlocking_ = isBlocking; }

uint32_t ULN2003::getStep() const { return currentStep_; }
uint32_t ULN2003::getToStep() const {return ((state_ == State::STOPPED) || (isInfinite_)) ? 0 : toStep_; }
ULN2003::Dir ULN2003::getDirection() const { return dir_; }

ULN2003::State ULN2003::getState() const { return state_; }

void ULN2003::move(const int32_t step, const bool isBlocking) {
    if ((state_ != State::STOPPED) || (step == 0) || (interval_ == 0)) { return; }
    state_ = State::MOVING;

    toStep_ = currentStep_ + step; //現在のステップから目標を設定する
    dir_ = (currentStep_ < toStep_) ? Dir::CW : Dir::CCW;
    startTimer();

    if (isBlocking) {
        while (state_ == State::MOVING) { update(); }
    }
}

void ULN2003::move(Dir dir) {
    if ((state_ != State::STOPPED) || (interval_ == 0)) { return; }
    state_ = State::MOVING;

    isInfinite_ = true;
    dir_ = dir;

    startTimer();
}

void ULN2003::stop() {
    if (state_ == State::STOPPED) { return; }
    decreaseStep_ = currentStep_;
    toStep_ = ((dir_ == Dir::CW) ? decreaseStep_ : -decreaseStep_) + ((calcSpeed(startSpeed_) - intervalAdj_) / acceleration_) / 2;
}

void ULN2003::pause() {
    if (state_ != State::MOVING) { return; }
    state_ = State::PAUSED;
    interrupter_.pause();
}

void ULN2003::resume() {
    if (state_ != State::PAUSED) { return; }
    state_ = State::MOVING;
    interrupter_.resume();
}

void ULN2003::update() {
    if (state_ != State::MOVING) { return; }

    if (interrupter_.update()) {
        processStep();
        accel();
    }
}

/* private */

void ULN2003::startTimer() {
    intervalAdj_ = (interval_ > delayCorrection_) ? interval_ - delayCorrection_ : 1;

    //加速度が0だったら加速を考慮せずにタイマーを開始
    if (acceleration_ == 0) {
        interrupter_.start((phase_ == Phase::HALF) ? (intervalAdj_) / 2 : intervalAdj_);
        return;
    }

    currentInterval_ = calcSpeed(startSpeed_); //RPM=1[min^(-1)]のときのインターバルから早めていく
    //減速を開始するステップを計算する(加速時よりシビアじゃないので2で割って時短)
    decreaseStep_ =  ((dir_ == Dir::CW) ? toStep_ : -toStep_) + ((currentInterval_ / acceleration_) / 2);

    accel();
}

void ULN2003::processStep() {
    if (!isInfinite_) {
        if (dir_ == Dir::CW) {
            if (currentStep_ >= toStep_) {
                brake();
                return;
            }
        } else {
            if (currentStep_ <= toStep_) {
                brake();
                return;
            }
        }
    }

    excitation((dir_ == Dir::CW) ? ++currentStep_ : --currentStep_);
}

void ULN2003::excitation(const uint32_t step) const {
    uint8_t current = 0;

    if (phase_ == Phase::HALF) {
        current = (PATTERNS >> (step % 2) * 16) >> ((step / 2) % 4 * 4);
    } else {
        current = (PATTERNS >> static_cast<uint8_t>(phase_) * 16) >> ((step % 4) * 4);
    }

    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(PINS[i], current & (1 << i));
    }
}
