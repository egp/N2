**Subject:** Tower overlap: end it at the bottom of the air dip instead of after a fixed time? (chart + questions)

Hi Tom,

Short version: right now the controller keeps both tower valves open together for a fixed 750 ms ("overlap") and then closes the first one. On our test day we measured what the air supply actually does during that time, and I'd like your opinion on closing the first valve **when the air pressure has bottomed out**, within a minimum and maximum time, instead of after a fixed time.

**Chart attached** (`overlap_for_tom_20261010.png`): the air pressure at the moment the second tower valve opens, from four recordings.

**What we measured (2026-10-09, your machine):**
* Air supply, steady: about 100-105 PSI.
* When the second valve opens, the air drops about **24 PSI**, to about 80 PSI.
* The bottom of the dip is about **400-500 ms** after the valve opens. By eye, it was close to 500 ms.
* It then climbs back; it is at about 90 PSI at 1 second, and about 100 PSI by 2 seconds.
* The first valve opening from fully closed is a bigger dip (about 35 PSI, to about 68-70 PSI, bottom at 200-300 ms), but that is the tower starting, not the overlap.
* Right-then-left and left-then-right look the same.
* In production the code allows the air to drop to 65 PSI before it stops the towers; it does not look at that during the first 3 s after a start, or the first 2 s after the second valve opens.

**Why there is an overlap (my understanding; please correct me):** with zero overlap the second tower would fill entirely from the air supply. With some overlap, the first (full) tower **pre-fills the second one** through the open valves, so the second tower needs less air from the supply and the first tower's pressure is recovered instead of vented. The air dip we see is the second tower drawing air; the bottom of the dip is about when that fill demand falls off, i.e. when the two towers have roughly equalised. So "bottom of the dip" is an indirect signal for "the pre-fill is done", measured on the supply side.

**What I'd like to do:**
The fixed 750 ms was chosen before we had any of this data. Proposal:
1. The overlap **must stay open at least 400 ms** (never closes the first valve earlier).
2. It **closes the first valve when the air pressure has hit the bottom of the dip** and started to come back (the air has to be about 1 PSI above its lowest point, and stay there for two readings 50 ms apart; a single noisy reading can't trigger it).
3. It **never stays open more than 800 ms**, even if no bottom has been seen (flat air, a noisy sensor, anything unexpected).

On our data that closes the first valve at about **600-700 ms**. (We detect the bottom with some delay because the pressure is flat there.)

**Questions for you:**
1. **Is that the purpose** (pre-fill the second tower from the first, saving supply air)? Is there any other reason for the overlap, for example keeping N2 flowing to the tank without a gap, or avoiding a pressure shock when a valve switches? If so, that sets how much we can shorten it.
2. **Is there a minimum overlap time you need for the towers' own sake**, regardless of the air pressure? Is 400 ms right, or should it be longer? Or even 750 ms as now?
3. **Is a maximum of 800 ms right?** Or should it be shorter or longer (we have measured one very slow recovery that would have needed 1 s)?
4. **Is it OK to close the first valve at the bottom of the dip**, when the supply is at its lowest but starting to come back? Or would you rather wait until it has recovered some of the way (say back to 90 PSI)? That would be a longer overlap, maybe 1 s.
5. **Is the 65 PSI low-air stop (90 PSI to restart) still reasonable** given a dip to about 80 PSI during every overlap? Our measured margin is about 15 PSI.
6. **Does the compressor or anything else on the same air line change the picture** (for example, the compressor starting while a tower valve opens)? We have not measured that yet.
7. **Can you give me the origin of the 750 ms**, if you remember? Was it measured, or just a number that worked?
8. **Tower pressure sensors (optimization only, not safety):** each tower has a pressure sensor that is not connected (its Arduino pin was reassigned to N2 LOW / N2 HIGH, which stay: they protect the compressor, so they must keep their pins). Reading the two tower pressures during the overlap would let us end it exactly when they equalise, which is the real goal; the supply-air dip is only an indirect signal. Facts about the Arduino: it has **only one free analog input** (A3). The other analog pins are air (A0), N2 LOW (A1), N2 HIGH (A2) and the two I2C lines (A4/A5, used by the LCD, LED, O2 sensor). So for two tower sensors we would need one of:
   * (a) a small external 4-channel ADC board (ADS1115, about $10) on the existing I2C bus, address 0x48. It can read both tower sensors, and it would not take any pin or disturb the safety inputs. If the bus fails the controller would just fall back to the air-dip rule.
   * (b) only one sensor, on A3: on one tower, or on a point common to both towers (for example the top manifold), if that is physically meaningful for equalisation.
   * (c) neither: keep the air-dip rule.
   Questions: what are those tower sensors (part number, range, 0.5-4.5 V?), where are they plumbed, and is the tower pressure actually worth having for the overlap? In any of (a)/(b) they would be optional: a missing or faulty tower sensor would never stop the machine, just fall back to the air-dip rule.

9. **Shall I buy the parts?** If you want the tower pressures, I would order **two DFRobot Gravity I2C ADS1115 16-bit ADC modules (part DFR0553, about $15.50 each)**: one for your panel, one for my bench to test the code before I bring it. It is the same maker as the O2 sensor, and it is a plug-in I2C board, with no pin or safety input taken. I will **not buy anything until you say yes**. Do you want me to order the pair and ship one to you, or would you rather get your own (any ADS1115 board at address 0x48 works)?
10. **Your pin spreadsheet:** could you send me the .csv (or spreadsheet) with your understanding of which Arduino pins connect to what (switches, valves, SSR, sensors, I2C, anything else)? I'll compare it line by line with what we verified on your machine on 2026-10-09 (TBS D0, TOB D1, LEFT D4, RIGHT D7, SSR D8, FLUSH D11, AIR A0, N2 LOW A1, N2 HIGH A2, A3 free, SDA/SCL A4/A5, LCD 0x23, LED 0x24, O2 0x74, RTC 0x68) and list every difference, so the wiring and the firmware say the same thing. The old V6/V7 sources had N2 LOW on A3 and N2 HIGH on A5, which we found were not how your panel is wired.

I'll implement the min/max/bottom-of-dip rule in the firmware once I have your answers, and we will verify it on your machine with the same kind of recording. Nothing changes on your machine until we do that together.

Thanks,
(your name)

*Attachments:* overlap_for_tom_20261010.png (also .svg). If useful: the raw data (CSV) for each recording is in the repo under `docs/results/`.
