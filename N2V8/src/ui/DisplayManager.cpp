#include "DisplayManager.h"

namespace n2 {

void DisplayManager::begin(uint32_t now, uint32_t lcdStartMs) {
  lcd_.enableHealing();  // see Lcd20x4::Healing
  if (lcdStartMs == 0) lcd_.begin(now);
  else lcdStart_.arm(now, lcdStartMs);
  led_.begin(now);
}

void DisplayManager::setOverride(const Screen& screen, const LedText& led, uint32_t now, uint32_t forMs) {
  override_ = true;
  haveShownNormal_ = false;   // when the override ends, the next normal screen goes out at once
  if (forMs > 0) timer_.arm(now, forMs);
  else timer_.clear();
  lcd_.setScreen(screen);
  led_.setText(led);
}

void DisplayManager::showNormal(const DisplayData& data, uint32_t now) {
  if (override_ && timer_.reached(now)) clearOverride();  // timed override (banner) ran out
  if (override_) return;
  cycle_.update(now, data.faultCount);
  const bool fault = cycle_.showFault();
  const Screen next = fault ? renderFault(data, cycle_.faultIndex()) : renderNormal(data, layout_);
  if (!haveShownNormal_ || !(next == shownNormal_)) {
    if (!haveShownNormal_ || static_cast<uint32_t>(now - shownNormalAt_) >= lcdMinChangeMs_) {   // output debounce: at most one change per lcdMinChangeMs_
      lcd_.setScreen(next);
      shownNormal_ = next;
      shownNormalAt_ = now;
      haveShownNormal_ = true;
    }
  }
  if (ledHold_ && (ledHoldUntil_.reached(now) || data.tbs)) ledHold_ = false;   // timed out, or the operator switched TBS on
  led_.setText(ledHold_ ? ledHoldText_ : renderLed(data, fault));
}

void DisplayManager::service(uint32_t now) {
  if (lcdStart_.reached(now)) {
    lcd_.begin(now);
    lcdStart_.clear();
  }
  lcd_.service(now);
  led_.service(now);
}

}  // namespace n2
