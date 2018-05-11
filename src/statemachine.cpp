#include "statemachine.hpp"

StateMachine::StateMachine() : m_timer(m_defTransTime) {
  m_state = LANE_KEEPING;
}

void StateMachine::nextState() {
  m_timer.tick();

  switch (m_state) {
    case LANE_KEEPING:
      if (m_timer.isElapsed() && m_recommend_lc_left) {
        m_state = PREPARE_LC_LEFT;
        m_timer.reset(m_defTransTime);
      } else if (m_timer.isElapsed() && m_recommend_lc_right) {
        m_state = PREPARE_LC_RIGHT;
        m_timer.reset(m_defTransTime);
      } else {
        // stay
      }
      break;

    case PREPARE_LC_LEFT:
      if (m_safe_to_finish) {
        m_state = LC_LEFT;
      } else if (m_timer.isElapsed()) {
        m_state = LANE_KEEPING;
        m_timer.reset(m_defTransTime);
      } else {
        // stay
      }
      break;

    case PREPARE_LC_RIGHT:
      if (m_safe_to_finish) {
        m_state = LC_RIGHT;
      } else if (m_timer.isElapsed()) {
        m_state = LANE_KEEPING;
        m_timer.reset(m_defTransTime);
      } else {
        // stay
      }
      break;

    case LC_LEFT:
      m_state = LANE_KEEPING;
      m_timer.reset(m_defTransTime);
      break;

    case LC_RIGHT:
      m_state = LANE_KEEPING;
      m_timer.reset(m_defTransTime);
      break;
  }
}