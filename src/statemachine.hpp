#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <iostream>

using namespace std;

class Timer {
 public:
  Timer(uint32_t ticks) {
    m_ticks = ticks;
    m_elapsed = false;
  }

  void tick() {
    if (m_ticks > 0) {
      --m_ticks;
    } else {
      m_elapsed = true;
    }
  }

  void reset(uint32_t ticks) {
      m_ticks = ticks;
      m_elapsed = false;
  }

  bool isElapsed() const { return m_elapsed; }

 private:
  uint32_t m_ticks;
  bool m_elapsed;
};

class StateMachine {
 public:
   enum State {
    LANE_KEEPING,
    PREPARE_LC_LEFT,
    PREPARE_LC_RIGHT,
    LC_LEFT,
    LC_RIGHT
  };

  StateMachine();
  void nextState();

  State getState() const
  {
      return m_state;
  }

 private:
  State m_state;
  Timer m_timer;

public:
  bool m_recommend_lc_left;
  bool m_recommend_lc_right;
  bool m_safe_to_finish;
  bool m_lc_finished;

  uint32_t m_defTransTime = 35;
};


#endif  // STATEMACHINE_H
