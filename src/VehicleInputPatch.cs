using HarmonyLib;

namespace T3taAutopilot
{
    /// <summary>
    /// The game calls EntityVehicle.MoveByAttachedEntity every frame for the local
    /// player sitting in the driver's seat and writes the player's input into
    /// EntityVehicle.movementInput. Our postfix runs right after and overwrites
    /// those fields with autopilot output while engaged.
    /// The physics step (PhysicsFixedUpdate) then consumes movementInput as usual,
    /// so the vehicle drives exactly like it would under player control.
    /// </summary>
    [HarmonyPatch(typeof(EntityVehicle), "MoveByAttachedEntity")]
    internal static class VehicleInputPatch
    {
        private static void Postfix(EntityVehicle __instance, EntityPlayerLocal _player)
        {
            AutopilotController.InjectVehicleInput(__instance, _player);
            Telemetry.OnVehicleFrame(__instance, _player);   // after our input, so it records what the vehicle got
        }
    }
}
