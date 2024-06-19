#ifndef MMZ_ULN_2003_H
#define MMZ_ULN_2003_H

#include <Arduino.h>
#include <OneShot.h>

class ULN2003 {
public:
    enum class State : uint8_t {
        STOPPED,
        MOVING,
        PAUSED
    };

    enum class Phase : uint8_t {
        ONE,
        TWO,
        HALF
    };

    enum class Dir : uint8_t {
        CW,
        CCW
    };

    ULN2003(const uint8_t pinA, const uint8_t pinB, const uint8_t pinC, const uint8_t pinD,
            const Phase phase=Phase::HALF, const uint16_t res=2048, uint32_t delayCorrection=0);

    /* アクセサー */

    uint8_t getStartSpeed() const;
    void setStartSpeed(const uint8_t speed);

    uint32_t getRPM() const;
    void setRPM(const uint32_t rpm);

    uint32_t getInterval() const;
    void setInterval(const uint32_t interval);

    ULN2003::Phase getPhase() const;
    void setPhase(const Phase phase);

    uint32_t getDelayCorrection() const;
    void setDelayCorrection(const uint32_t delayCorrection);

    uint16_t getAcceleration() const;
    void setAcceleration(const uint16_t a);

    uint32_t getStep() const;
    uint32_t getToStep() const;
    ULN2003::Dir getDirection() const;

    bool isBlocking() const;
    void setBlocking(const bool isBlocking);

    ULN2003::State getState() const;

    /* 回転 */

    inline void move(const int32_t step)  { move(step, isBlocking_); }
    void move(const int32_t step, const bool isBlocking);
    void move(Dir dir);

    inline void moveByRev(const int32_t rev) { moveByRev(rev, isBlocking_); }
    inline void moveByRev(const int32_t rev, const bool isBlocking) {
        move((phase_ == Phase::HALF) ? rev * static_cast<int32_t>(resolution_) * 2 : rev * static_cast<int32_t>(resolution_), isBlocking);
    }

    void stop();

    void pause();
    void resume();

    inline void brake() {
        state_ = State::STOPPED;
        isInfinite_ = false;

        interrupter_.cancel();
        for (uint8_t pin : PINS) {
            digitalWrite(pin, LOW);
        }
    }

    void update();

    /* ユーティリティ系 */

    //toggle~系のメソッドは実行した結果回転したときにtrueを返す

    inline void blockWhileMoving() const { while (state_ == State::MOVING) { update(); } }

    inline bool toggleMoveStop(const int32_t step) { return toggleMoveStop(step, isBlocking_); }
    inline bool toggleMoveStop(const int32_t step, const bool isBlocking) {
        if (state_ == State::MOVING) {
            stop();
            return false;
        } else {
            move(step, isBlocking);
            return true;
        }
    }
    inline bool toggleMoveStopByRev(const int32_t step) { return toggleMoveStop(step, isBlocking_); }
    inline bool toggleMoveStopByRev(const int32_t step, const bool isBlocking) {
        if (state_ == State::MOVING) {
            stop();
            return false;
        } else {
            moveByRev(step, isBlocking);
            return true;
        }
    }
    inline bool togglePauseResume() {
        if (state_ == State::PAUSED) {
            resume();
            return true;
        } else {
            pause();
            return false;
        }
    }

private:
    inline uint32_t calcSpeed(const uint16_t sp) { return 60L * (1000L * 1000L) / resolution_ / sp; }

    inline void accel() {
        //加速度が0(=無効)だったらスキップする
        if (acceleration_ != 0) {
            if (dir_ == Dir::CW) {
                //加減速開始判定
                if (currentStep_ >= decreaseStep_) {
                    currentInterval_ += acceleration_;
                } else {
                    checkInterval();
                }
            } else {
                if (currentStep_ < decreaseStep_) {
                    currentInterval_ += acceleration_;
                } else {
                    checkInterval();
                }
            }

            interrupter_.start((phase_ == Phase::HALF) ? currentInterval_ / 2 : currentInterval_);
        } else {
            interrupter_.start(intervalAdj_);
            return;
        }
    }

    inline void checkInterval( ) {
        if (currentInterval_ > intervalAdj_) { currentInterval_ -= acceleration_; }
        if (currentInterval_ < intervalAdj_) { currentInterval_ = intervalAdj_; }
    }

    void startTimer();

    void processStep();
    void excitation(const uint32_t step) const;

    const uint8_t PINS[4];
    const uint16_t resolution_;

    uint8_t startSpeed_;
    int32_t currentStep_, toStep_;
    uint32_t interval_, delayCorrection_, intervalAdj_;
    uint32_t acceleration_, currentInterval_, decreaseStep_;

    bool isInfinite_, isBlocking_;

    State state_;
    Phase phase_;
    Dir dir_;

    OneShot interrupter_;

    //励磁パターン
    static constexpr uint32_t PATTERNS =
        //一相励磁
          (static_cast<uint32_t>(0b0001) << (0*4))
        | (static_cast<uint32_t>(0b0010) << (1*4))
        | (static_cast<uint32_t>(0b0100) << (2*4))
        | (static_cast<uint32_t>(0b1000) << (3*4))
        //二相励磁
        | (static_cast<uint32_t>(0b0011) << (4*4))
        | (static_cast<uint32_t>(0b0110) << (5*4))
        | (static_cast<uint32_t>(0b1100) << (6*4))
        | (static_cast<uint32_t>(0b1001) << (7*4));
};

#endif
