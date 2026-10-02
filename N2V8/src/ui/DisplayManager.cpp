#include "DisplayManager.h"

namespace n2 {

void DisplayManager::begin(uint32_t now) {
  lcd_.begin(now);
  led_.begin(now);
}

void DisplayManager::setOverride(const Screen& screen, const LedText& led, uint32_t now, uint32_t forMs) {
  override_ = true;
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
  lcd_.setScreen(fault ? renderFault(data, cycle_.faultIndex()) : renderNormal(data, layout_));
  led_.setText(renderLed(data, fault));
}

void DisplayManager::service(uint32_t now) {
  lcd_.service(now);
  led_.service(now);
}

}  // namespace n2
