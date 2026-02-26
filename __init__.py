import bpy
from bpy.types import Operator, Panel
import math
import statistics  
    
def estimate_previous_velocity(fcurve, frame, num_samples=3, std_threshold=0.1, fallback=1e-3):
    # Returns estimated slope of existing keyframe by sampling previous curve

    current_value = fcurve.evaluate(frame)

    # Collect backward samples
    samples = []
    for i in range(1, num_samples + 1):
        prev_frame = frame - i
        prev_value = fcurve.evaluate(prev_frame)
        samples.append((prev_frame, prev_value))

    # Compute slopes
    slopes = []
    for prev_frame, prev_value in samples:
        dx = frame - prev_frame
        dy = current_value - prev_value
        if dx != 0:
            slopes.append(dy / dx)

    if not slopes:
        return 0.0

    # Sign consistency check
    signs = [1 if s > 0 else -1 if s < 0 else 0 for s in slopes]
    consistent_sign = all(s == signs[0] for s in signs)

    # Standard deviation check
    slope_std = statistics.pstdev(slopes) if len(slopes) > 1 else 0.0

    if abs(statistics.median(slopes)) > fallback:
        # If unstable, return most recent slope (slopes[0])
        if not consistent_sign or slope_std > std_threshold:
            return slopes[0]
        
        # Otherwise return median slope
        return statistics.median(slopes)
    else:
        return 1
    
def find_keyframe_at_frame(fcurve, frame):
    target = int(frame)

    for kp in fcurve.keyframe_points:
        if int(kp.co[0]) == target:
            return kp

    return None

    
def build_selected_map(fcurves):
    """
    Returns:
        selected_map: { fcurve: [(frame, value)] }
    """
    selected_map = {}

    for fc in fcurves:
        selected = [kp for kp in fc.keyframe_points if kp.select_control_point]

        # Abort PER F-CURVE if more than one keyframe is selected
        if len(selected) > 1:
            self.report({'ERROR'}, "Select exactly one keyframe per fcurve")
            return {'CANCELLED'}      

        # Store exactly one selected keyframe
        if len(selected) == 1:
            kp = selected[0]
            selected_map[fc] = [(kp.co[0], kp.co[1])]

    return selected_map
    
def reselect_keys(selected_map):
    for fc, key_list in selected_map.items():
        for frame, value in key_list:
            # Find the actual keyframe object after decimation
            for kp in fc.keyframe_points:
                if abs(kp.co[0] - frame) < 1e-6:
                    kp.select_control_point = True
                    break
    
    
class VelocityOperator(Operator):
    bl_idname = "graph.extend_curve_velocity"
    bl_label = "Overshoot Selected Curve Velocity"
    bl_options = {'REGISTER', 'UNDO'}

    timing: bpy.props.IntProperty(
        name="Timing",
        description="length of an oscillation",
        default=5,
        min=1
    )
    
    decay: bpy.props.FloatProperty(
        name="Decay",
        description="How rapidly the overshoot decays to 0",
        default=0.5,
        min=0,
        soft_min=0.3,
        max=1
    )

    amplitude: bpy.props.FloatProperty(
        name="Amplitude",
        description="Scale factor for the velocity extension",
        default=1.0,
    )    

    overwrite_keyframes: bpy.props.BoolProperty(
        name="Overwrite conflicting keyframes",
        description="Choose whether you are okay with future keyframes being overwritten in your animation.",
        default=True
    )
    
    
    def invoke(self, context, event):
        # Show popup dialog with all properties
        return context.window_manager.invoke_props_dialog(self)      
    
    
    def execute(self, context):
        processed = 0
        
        # Adapted from EaseIt: Gather selected fcurves
        fcurves = []

        # Try Graph Editor context first
        if hasattr(context, 'selected_visible_fcurves') and context.selected_visible_fcurves:
            fcurves = context.selected_visible_fcurves
        elif hasattr(context, 'active_editable_fcurve') and context.active_editable_fcurve:
            fcurves = [context.active_editable_fcurve]

        if not fcurves:
            self.report({'ERROR'}, "No F-Curves found")
            return {'CANCELLED'}
            
        map = build_selected_map(fcurves)
        for fcurve in fcurves:

            # Find selected keyframe
            selected = [kf for kf in fcurve.keyframe_points if kf.select_control_point]
            if len(selected) != 1:
                continue
            
            kf = selected[0]
            kf_frame = kf.co[0]
            kf_value = kf.co[1]
            kf.select_control_point = False            

            # Compute velocity at the keyframe
            velocity = estimate_previous_velocity(fcurve, kf_frame)
            velocity *= self.amplitude
            
            # Identify owner
            owner = fcurve.id_data
            fcurve_key = f"{fcurve.data_path}_{fcurve.array_index}"

            # Read previous history
            last_kf = owner.get(f"{fcurve_key}_keyframe", None)
            last_dur = owner.get(f"{fcurve_key}_duration", None)

            # Delete old baked keys
            if last_kf is not None and last_dur is not None:
                end_frame = last_kf + last_dur
                indices = []

                for i, kp in enumerate(fcurve.keyframe_points):
                    frame = kp.co[0]
                    if last_kf < frame <= end_frame:
                        indices.append(i)

                for i in reversed(indices):
                    fcurve.keyframe_points.remove(fcurve.keyframe_points[i])

            # Spring parameters
            omega = (2 * math.pi) / (self.timing + 1)
            k = omega * omega
            decay_remapped = self.decay ** 3  # your existing non-linear remap
            c = 2 * decay_remapped * math.sqrt(k) if k > 0 else 0
            
            # Simulation state
            x = 0.0
            v = velocity
            
            initial_magnitude = abs(velocity) + 1e-6
            epsilon = initial_magnitude * 1e-2
            
            max_iter = 500
            frame = kf_frame
            iter_count = 0
            substeps = 50
            dt = 1.0 / substeps
            
            # Delay damping until first peak (velocity sign change)
            damping_active = False
            prev_v_sign = 1 if v >= 0 else -1 # initial direction
            
            # Simulate forward until motion dies
            while (abs(x) > epsilon or abs(v) > epsilon) and iter_count < max_iter:
                for _ in range(substeps):
                    c_current = c if damping_active else 0.0
                    a = -k * x - c_current * v
                    v += a * dt
                    x += v * dt
                
                # Detect first peak and start decay function after
                current_v_sign = 1 if v >= 0 else -1
                if not damping_active and current_v_sign != prev_v_sign and current_v_sign != 0:
                    damping_active = True
                prev_v_sign = current_v_sign

                next_frame = frame + 1
                existing = find_keyframe_at_frame(fcurve, next_frame)

                # Abort baking if overwrite is disabled and a keyframe exists
                if existing and not self.overwrite_keyframes:
                    break

                frame = next_frame
                value = kf_value + x

                # Overwrite existing keyframe if allowed
                if existing and self.overwrite_keyframes:
                    fcurve.keyframe_points.remove(existing)

                kp = fcurve.keyframe_points.insert(frame=frame, value=value)
                kp.interpolation = 'BEZIER'

                iter_count += 1

            
            if iter_count >= max_iter:
                self.report({'WARNING'}, "Simulation capped — adjust decay or timing")


            # After decimate:
            # Find the keyframe at kf_frame again
            new_kf = None
            for kp in fcurve.keyframe_points:
                if int(kp.co[0]) == kf_frame:
                    new_kf = kp
                    new_kf.select_control_point = False
                    break
    
                  
            # After baking:
            owner[f"{fcurve_key}_keyframe"] = kf_frame
            owner[f"{fcurve_key}_duration"] = (frame - kf_frame)
            
            fcurve.update()
            processed += 1
        
     
        
        graph_area = next((a for a in bpy.context.screen.areas if a.type == 'GRAPH_EDITOR'), None)
        if graph_area:
            kf.select_control_point = False
            with bpy.context.temp_override(area=graph_area):
                try:
                    bpy.ops.graph.decimate(mode='ERROR', remove_error_margin=0.01)
                except Exception as ex:
                    print("Decimate failed:", ex)         
        
        #for fcurve in fcurves:
        # ─── Final cleanup: deselect + smart handle types ────────────────────────
        for fcurve in fcurves:
            for kp in fcurve.keyframe_points:
                was_selected = kp.select_control_point  # remember before we deselect

                # Deselect everything
                kp.select_control_point = False
                kp.select_left_handle = False
                kp.select_right_handle = False

                # If this keyframe is (or was) selected → force AUTO_CLAMPED handles
                # This catches both: original selected key AND any re-selected via reselect_keys()
                if was_selected or kp.select_control_point:  # second check is safety
                    if kp.interpolation == 'BEZIER':
                        kp.handle_left_type = 'AUTO_CLAMPED'
                        kp.handle_right_type = 'AUTO_CLAMPED'

            fcurve.update()  # Important after handle changes
                
        reselect_keys(map)
            
        if processed == 0:
            self.report({'ERROR'}, "Select exactly one keyframe per fcurve")
            return {'CANCELLED'}            
        
        self.report({'INFO'}, f"Processed {processed} curves")
        return {'FINISHED'}

# ────────────────────────────────────────────────
# Panel
# ────────────────────────────────────────────────

class GRAPH_PT_extend_velocity(Panel):
    bl_label = "Momentum"
    bl_idname = "GRAPH_PT_extend_velocity"
    bl_space_type = 'GRAPH_EDITOR'
    bl_region_type = 'UI'
    bl_category = 'Momentum'  # appears in Tool tab of N-panel

    @classmethod
    def poll(cls, context):
        return context.space_data.type == 'GRAPH_EDITOR'

    def draw(self, context):
        layout = self.layout
        layout.operator("graph.extend_curve_velocity", text="Overshoot selected keyframe")
# ────────────────────────────────────────────────
# Registration
# ────────────────────────────────────────────────

_classes = (
    VelocityOperator,
    GRAPH_PT_extend_velocity,
)

def register():
    for cls in _classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()