
## Checks and bug fixes

[*] Check deluge drum trigger input 
[*] Check triggers for plaits models (can we trigger without lpg?)
[ ] Preset saving for plaits mode


# Optimisations 

[ ] Try adding back EQ and soft limiting
[*] Optimise LPG 


## Improvements to existing features

[x] Alternate wavetable browsing path
[x] decouple LPG params from lfo params 
[ ] Sample and hold on plaits params per trigger 
[x] Figure out plaits control mapping and UI (how do we avoid jumps?)
[ ] Better colours for plaits voices 
[x] Better LPG mode selection 


## new feature

[*] Move spread from integer to float, adding uneven spacing
[x] LFO phase options - strum mode, even spread 
[ ] round robbin triggering
[ ] midi input! 
[ ] Euclidean rhythms for LPG modes


## Control Scheme

For plaits, we have the three main params:  timbre, morph and harmonics. 
That leaves the central encoder free.  We can use that for the model select, and move all the voices to one SWN slot, or find another use for it. 

We currently don't map any modulation to the plaits models, other than the main CV inputs that affect all voices. 


For swn voices, I don't love the sphere browsing, though maybe it's better now the dram cahce issue is fixed. I'd prefer to be able to morph through in a linear way, and have this mapped to the lpg decay    
We have three params mapping zero to one.
We could map them to modulation 