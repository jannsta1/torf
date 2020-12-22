Header header
string state
float32 score
int32 index                           # the index of the best score (in the storage bank of images)
int32 stored_qty
int32 sweep_index                     # how many sweeps have we don or, which sweep are we on?
float32 best_score_previous_run       # waht was our best score on the previous sweep?


## things we could add:
# longitudinal_slowdown_factor
# lateral_slowdown_factor
# on_last_sweep

## things we shouldn't add (since they are already logged in other data):
# float32 x_pos
# float32 y_pos
# float32 z_pos