# The message format for attaching a event script to a marker (hence called active marker in the code)

uint8 BECOME_ACTIVE=0      # When the marker gains focus
uint8 BECOME_INACTIVE=1    # When the marker looses focus
uint8 LEFT_CLICK=2         # Left clicking on a marker that is active
uint8 PROPERTY_SCRIPT=3    # For scripts added to the right click menu of CRI
uint8 PICK_OPTION=4        # A 'pick' menu script 

uint8 ADD_OR_MODIFY=0
uint8 DELETE=1

string ns                  # Namespace to place this object in... used in conjunction with id to specify a unique name for the target marker
int32 id                   # object ID useful in conjunction with the namespace for unique marker name (if the id is -1, the script applies to the entire namespace)
uint8 action			   # Add, or delete?
string script_name         # Unique name for the script with regard to the object
uint32 trigger_event_type
string code                # Leave empty if code is to be loaded from file "filename"
string filename            # Leave empty if code contains the ... code

