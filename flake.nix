{
  description = "ZMK firmware development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    zephyr.url = "git+ssh://git@github.com/FrameworkComputer/zephyr-private?ref=daisy-zephyr4.1";
    zephyr.flake = false;

    zephyr-hid-touchpad-module.url = "git+ssh://git@github.com/FrameworkComputer/zephyr-hid-touchpad-module?ref=passthrough";
    zephyr-hid-touchpad-module.flake = false;

    zephyr-nix.url = "github:nix-community/zephyr-nix";
    zephyr-nix.inputs.nixpkgs.follows = "nixpkgs";
    zephyr-nix.inputs.zephyr.follows = "zephyr";

    west2nix.url = "github:adisbladis/west2nix";
    west2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, zephyr-nix, ... }@inputs: let
    system = "x86_64-linux";

    pkgs = import nixpkgs {
      inherit system;
      overlays = [
        # python310 was removed from nixpkgs-unstable; alias to python312
        (final: prev: { python310 = final.python312; })
      ];
    };

    zephyr = pkgs.callPackage zephyr-nix {
      zephyr-src = inputs.zephyr;
      pyproject-nix = zephyr-nix.inputs.pyproject-nix;
    };

    # Build SDK 0.17.0 to match Zephyr 4.1.0 (zephyr-nix ships 0.17.4 which is incompatible)
    sdk_0_17_0 = pkgs.callPackage
      (import "${zephyr-nix}/sdk.nix" (pkgs.lib.importJSON ./sdk-0.17.0.json))
      { python3 = pkgs.python312; };

    callPackage = pkgs.newScope (pkgs // {
      inherit zephyr sdk_0_17_0;
      west2nix = pkgs.callPackage inputs.west2nix.lib.mkWest2nix { };
      zephyr-src = inputs.zephyr;
      touchpad-module-src = inputs.zephyr-hid-touchpad-module;
    });
  in {
    packages.${system} = {
      default = callPackage ./default.nix {};
      xiao_ble = callPackage ./default.nix { board = "xiao_ble"; };
      nrf52840dk = callPackage ./default.nix { board = "nrf52840dk/nrf52840"; };
    };

    lib.mkFirmware = { board, shield ? null }:
      callPackage ./default.nix { inherit board shield; };

    devShells.${system}.default = callPackage ./shell.nix {};
  };
}
