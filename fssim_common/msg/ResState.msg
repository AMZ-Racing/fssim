# State of the three buttons on the RES
bool emergency 					# 0 = OK, 1 = Emergency
bool on_off_switch 				# 0 = 0, 1 = 1 (Comments couldn't be more helpful :p )
bool push_button 				# 1 = pressed
# RES will trigger emergency 200ms after this becomes 1 (unless communication is recovered)
bool communication_interrupted 	# 0 = OK, 1 = Interrupted
