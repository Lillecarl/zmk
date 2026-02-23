{
  description = "ZMK firmware development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    zephyr.url = "github:zmkfirmware/zephyr/v4.1.0+zmk-fixes";
    zephyr.flake = false;

    zephyr-nix.url = "github:nix-community/zephyr-nix";
    zephyr-nix.inputs.nixpkgs.follows = "nixpkgs";
    zephyr-nix.inputs.zephyr.follows = "zephyr";
  };

  outputs = { self, nixpkgs, zephyr-nix, ... }@inputs: let
    pkgs = import nixpkgs {
      system = "x86_64-linux";
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
  in {
    devShells.x86_64-linux.default = pkgs.mkShell {
      packages = [
        ((sdk_0_17_0.sdk.override {
          targets = [
            "arm-zephyr-eabi"
          ];
        }).overrideAttrs {
          autoPatchelfIgnoreMissingDeps = [ "libpython3.10.so.1.0" ];
        })
        zephyr.pythonEnv
        zephyr.hosttools-nix
        pkgs.cmake
        pkgs.ninja
      ];
    };
  };
}
