{
  description = "Pinned minimal LOTUSim-xdyn open-loop bench";
  inputs.lotusim.url = "github:naval-group/LOTUSim/2034754b2140006fd615bcb1c420d6a7c83f0642";
  outputs = { self, lotusim }: let
    system = "x86_64-linux";
    overlay = lotusim.inputs.nix-ros-overlay;
    pkgs = import overlay.inputs.nixpkgs { inherit system; overlays = [ overlay.overlays.default ]; };
    ros = pkgs.rosPackages.jazzy;
    messages = lotusim.packages.${system}.messages;
  in {
    devShells.${system}.default = pkgs.mkShell {
      packages = [ pkgs.python3 pkgs.python3Packages.websockets pkgs.python3Packages.shapely messages
        lotusim.inputs.lxdyn.packages.${system}.xdyn lotusim.packages.${system}.lotusim ]
        ++ (with ros; [ rclpy action-msgs builtin-interfaces geographic-msgs geometry-msgs
          sensor-msgs std-msgs std-srvs rosidl-default-runtime ]);
      shellHook = ''
        export PYTHONPATH="${messages}/${pkgs.python3.sitePackages}:$PYTHONPATH"
        export AMENT_PREFIX_PATH="${messages}:$AMENT_PREFIX_PATH"
        export LD_LIBRARY_PATH="${messages}/lib:$LD_LIBRARY_PATH"
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
      '';
    };
  };
}
