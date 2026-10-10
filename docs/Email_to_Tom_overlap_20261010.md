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

**What I'd like to do:**
The fixed 750 ms was chosen before we had any of this data. Proposal:
1. The overlap **must stay open at least 400 ms** (never closes the first valve earlier).
2. It **closes the first valve when the air pressure has hit the bottom of the dip** and started to come back (the air has to be about 1 PSI above its lowest point, and stay there for two readings 50 ms apart; a single noisy reading can't trigger it).
3. It **never stays open more than 800 ms**, even if no bottom has been seen (flat air, a noisy sensor, anything unexpected).

On our data that closes the first valve at about **600-700 ms**. (We detect the bottom with some delay because the pressure is flat there.)

**Questions for you:**
1. **Why is there an overlap at all?** Is it to keep gas flowing to the N2 tank, to equalise pressure between the towers before the one changes, to avoid slamming a valve, or something else? The answer decides how much we can shorten or lengthen it.
2. **Is there a minimum overlap time you need for the towers' own sake**, regardless of the air pressure? Is 400 ms right, or should it be longer? Or even 750 ms as now?
3. **Is a maximum of 800 ms right?** Or should it be shorter or longer (we have measured one very slow recovery that would have needed 1 s)?
4. **Is it OK to close the first valve at the bottom of the dip**, when the supply is at its lowest but starting to come back? Or would you rather wait until it has recovered some of the way (say back to 90 PSI)? That would be a longer overlap, maybe 1 s.
5. **Is the 65 PSI low-air stop (90 PSI to restart) still reasonable** given a dip to about 80 PSI during every overlap? Our measured margin is about 15 PSI.
6. **Does the compressor or anything else on the same air line change the picture** (for example, the compressor starting while a tower valve opens)? We have not measured that yet.
7. **Can you give me the origin of the 750 ms**, if you remember? Was it measured, or just a number that worked?

I'll implement the min/max/bottom-of-dip rule in the firmware once I have your answers, and we will verify it on your machine with the same kind of recording. Nothing changes on your machine until we do that together.

Thanks,
(your name)

*Attachments:* overlap_for_tom_20261010.png (also .svg). If useful: the raw data (CSV) for each recording is in the repo under `docs/results/`.
