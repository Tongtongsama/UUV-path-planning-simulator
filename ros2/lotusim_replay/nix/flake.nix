{
  description = "Python ROS environment for LOTUSim frozen Gazebo replay";
  inputs.lotusim.url = "github:naval-group/LOTUSim";
  outputs = { self, lotusim }: let
    system = "x86_64-linux";
    overlay = lotusim.inputs.nix-ros-overlay;
    pkgs = import overlay.inputs.nixpkgs {
      inherit system;
      overlays = [ overlay.overlays.default ];
    };
    ros = pkgs.rosPackages.jazzy;
    messages = lotusim.packages.${system}.messages;
  in {
    devShells.${system}.default = pkgs.mkShell {
      # The standalone colcon messages output does not propagate its Python
      # message dependencies into mkShell. Include the runtime closure here.
      packages = [ pkgs.python3 messages ] ++ (with ros; [
        rclpy
        action-msgs
        builtin-interfaces
        geographic-msgs
        geometry-msgs
        sensor-msgs
        std-msgs
        std-srvs
        rosidl-default-runtime
      ]);
      shellHook = ''
        export PYTHONPATH="${messages}/${pkgs.python3.sitePackages}:$PYTHONPATH"
        export AMENT_PREFIX_PATH="${messages}:$AMENT_PREFIX_PATH"
        export LD_LIBRARY_PATH="${messages}/lib:$LD_LIBRARY_PATH"
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
      '';
    };
  };
}
