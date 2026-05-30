# MVN Blender Plugin

The MVN Blender Plugin receives data from MVN Animate and provides a platform for recording, retargeting, and manipulating MVN
data in Blender.

Supports Blender version 3.6-4.0

Full user documentation can be found in the `docs` folder.

## Installation

1. Open Blender. `Navigate to Edit > Preferences > Add-ons`

  <p align="center">
    <img src="/images/installation_01.png" alt="None">
  </p>

2. Click `Install` in the top right and select the directory of the add-on files. This will add the contents to default
   location: `C:\Users\<USERNAME>\AppData\Roaming\Blender Foundation\Blender\3.x\scripts\addons`

3. Check the box next to “Motion Capture: MVN Live Plugin” to enable to add-on. The add-on will now be enabled for this
   version of Blender and can be disabled or removed at any time from this menu.

  <p align="center">
    <img src="/images/installation_02.png" alt="None">
  </p>

- **NOTE:** For this default workflow, Blender add-ons are local to the user and specific to the Blender version. If
  using a different user, PC, or Blender version (i.e. 3.6 -> 3.3 or 4.0), the add-on will need to be installed again.
- **NOTE:** Before removing the add-on, be sure to stop any active recording or streaming in progress to prevent issues.

- **More info:** [Blender Add-ons](https://docs.blender.org/manual/en/3.6/editors/preferences/addons.html)

## UI

The UI for the add-on is accessed through the View3D Sidebar under a tab called `Xsens`. This can be revealed by clicking the arrow in the top right of the viewport window or by using the Blender default hotkey `N`.

  <p align="center">
    <img src="/images/ui_01.png" alt="None">
  </p>

  <p align="center">
    <img src="/images/ui_02.png" alt="None">
  </p>

## Streaming

1. Set the `Address` and `Port` settings as necessary

   - If MVN is on the same system, either `localhost` or `127.0.0.1` can be used for the IP address.

2. Click `Start Stream`
    - Each MVN actor will be created as its own armature named “MVN:[actor name]”. If the actor has finger data, finger bones will be included in the respective armature.
    - **The Start/Stop Stream options can only be selected if the viewport is set to “Object Mode”**

      <p align="center">
        <img src="/images/streaming_01.png" alt="None">
      </p>
   
   - Props will be created as empties named “MVN:[actor name]_Prop[#]”. They will also be added to the respective MVN armature’s hierarchy as bones named “Prop[#]” and will be parented within the hierarchy depending on the configuration in MVN.
   - **The bones added for props will always be created at a consistent orientation and roll (pointing up similar to the spine) regardless of where the prop is attached to in MVN.**

   - Vive objects will be created as empties named “MVN:Objects_HTC Vive [object type]: [object name]”.

      <p align="center">
        <img src="/images/streaming_02.png" alt="None">
      </p>

   - The object name will be whatever is displayed in MVN’s viewport.

3. `Scene Scale` allows users to uniformly scale the MVN armatures with respect to Blender global space in order to match the scale of their own armatures.
   - **At the moment, only armatures are affected by “Scene Scale”. Empties are not affected.**

4. All MVN Objects created through the add-on will be added to a new “MVN Collection”.

      <p align="center">
        <img src="/images/streaming_03.png" alt="None">
      </p>

**NOTES:**
   - Any time the streamer is started (including stopped and restarted), all MVN armatures will be recreated. This means any references to the armatures will need to be restored (user constraints/parenting, `Source:` referencing in the retargeter, etc.). All settings to reference names are still maintained in Blender, so if the reference is reset to an object with the same name, everything should work as before (i.e. constraint settings will stay the same, bone remapping/IK settings will stay configured).
   - Any changes made to the MVN objects (changing, renaming, or deleting object/bones, etc.) will cause that object to stop receiving stream data unless the changes are reverted. Objects can also be easily recreated by toggling the stream.
   - If the name of an object that would be created by the add-on already exists in the scene, the existing object will be deleted and replaced by that of the add-on (i.e. the MVN actor’s name is `MVN System 1` and there is already an object called `MVN:MVN System 1` that exists in Blender when the stream is started). Although this would be a rather specific scenario for a user to encounter, this would still mean a loss of user data, so this should be made clear.
   - When opening/creating a new file, the user should stop the streamer/recorder first before doing so to prevent any issues. Otherwise, Blender may require a restart for the add-on to function properly again.
   - `Character Meta Data` and `Scaling Data` are required options in MVN’s network streamer for the add-on to be able to stream properly.

## Retargeting

1. Clicking the text box next to “Target :” will reveal a dropdown menu of all non-MVN armatures detected in the scene. Select the desired target armature.

      <p align="center">
        <img src="/images/retargeting_01.png" alt="None">
      </p>

2. Clicking the `Source :` text box to choose an MVN armature to act as the source for the target.

      <p align="center">
        <img src="/images/retargeting_02.png" alt="None">
      </p>

- **All armatures in the scene will appear in the source dropdown menu. This is for the purpose of offline retargeting explained later.**

3. Before retargeting can occur, the target character needs to have a T-pose applied to the add-on (same T-pose used in other plugins, including for fingers). The T-pose is referenced from the target armature’s current pose in Blender’s `Pose Mode`.

   - If the armature is not already in a T-pose, select the target armature and go into `Pose Mode` by changing the mode in the top left corner of the viewport or by using the Blender default hotkeys `ctrl + tab`.
    
        <p align="center">
          <img src="/images/retargeting_03.png" alt="None">
        </p>

   - The armature can then be rotated as needed. Rotations do not need to be applied (applying pose in Blender) or keyed. However, once the T-pose is applied in the add-on, any changes made to the target armature in “Pose Mode” will add offsets to the retargeting.

        <p align="center">
          <img src="/images/retargeting_04.png" alt="None">
        </p>

   - **If there are keyframes on the bones, any rotations made by the user will be lost if they are not keyed as well or if the previous keyframes are not deleted.**

4. Once the target armature is in the desired T-Pose, click “Apply T-Pose”. The T-Pose can be edited and reapplied at any time.

   - Minor adjustments can be made to the retargeting to help with specific issues (such as the arms clipping into the body). However, if large adjustments need to be made, it is better to do these by editing the T-Pose and then reapplying.

5. Once the T-Pose is applied, “Bone Remapping” becomes available. The left side shows all bones in the target armature and the right side contains text boxes for selecting the MVN source armature bone to map to.

      <p align="center">
        <img src="/images/retargeting_05.png" alt="None">
      </p>

   - `Auto Map Bones` uses default naming conventions to attempt to automatically map MVN source bones based on the target bones’ names.
     - Naming conventions used: Xsens, Blender, HIK, Unreal, 3DS Max/Biped
   - Bone mappings can be saved and loaded using the buttons below the bone mapping section. Mappings are saved as .csv files.
     - **The default location set up by the add-on for saving and loading mappings is a folder created in the user’s AppData folder. If the add-on is removed (not just disabled), this folder and its contents will be deleted along with the add-on files.**
   - **If the “Source” changes to an MVN armature that does not have the same bones as the previous source (e.g. switching from a source with fingers to one without), any bones mapped to those missing fingers will be cleared from the bone remapping. Users should be sure to save mappings as needed to prevent wasted work.**
   - **Automapping or loading a bone map will overwrite any manual changes in the bone remapping.**

6. Once all bones are mapped properly, the target armature should be driven by the source through FK retargeting.

   - The target armature can be repositioned as needed in `Object Mode`.
   - FK retargeting is achieved through a combination of transformation constraints and custom drivers in Python which are added to each bone in the target armature’s hierarchy depending on what was configured in the bone mapping. Each transformation constraint copies the respective bone’s rotation (with the exception of the pelvis which has an additional constraint to also track location).

    <p align="center">
      <img src="/images/retargeting_06.png" alt="None">
    </p>

7. The FK retargeting constraints can be reset at any time by remapping bones, toggling the applied T-Pose, or resetting the retargeting source. This will only clear out constraints following the naming convention used by the add-on (“MVN\_”), so user created constraints will be left alone as long as they do not follow this naming convention.

8. **For finger retargeting, if there are no metacarpal bones in the target character’s hands, the finger bones will require an offset aside from just the finger T-Pose. This behavior is also consistent from experience with other plugins, and seems to be due to the converted rotation data from MVN for the fingers being affected by the metacarpal data since the fingers are children of the metacarpals whereas that data is lost in retargeting due to not having metacarpals. One workaround for a user would be to add dummy metacarpal bones to their target hands for bone remapping. These bones would not need to be skinned/weighted to the mesh, but they would need to be parents of the respective finger bones.**

## Recording

Recordings in the add-on will record any motion done by the source armatures and will save that data as actions (animation files) in Blender. These actions can then be used to drive the source armatures which can still be retargeted to a user’s armature.

1. `Take :` will set the name of the action, and the number next to the text box will append a take number to the name.

      <p align="center">
        <img src="/images/recording_01.png" alt="None">
      </p>

2. The framerate for the recording can be set which determines how many keyframes will be recorded to the action.

      <p align="center">
        <img src="/images/recording_02.png" alt="None">
      </p>

   - **This setting is independent of Blender’s viewport playback fps, so the recorded framerate may not appear as expected unless Blender’s viewport fps also matches.**

     <p align="center">
       <img src="/images/recording_03.png" alt="None">
     </p>

3. The add-on can trigger a recording in MVN through the UDP Remote Control function which will also control the take name and take number.
