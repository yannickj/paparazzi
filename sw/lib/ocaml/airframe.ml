(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * Airframe module for parsing XML config files
 *)

module OT = Ocaml_tools

module Autopilot = struct

  type t = { name: string; freq: float option; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("autopilot", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          freq = OT.assoc_opt_map "freq" attrs float_of_string;
          xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Include = struct

  type t = { href: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("include", [("href", href)], []) as xml -> { href; xml }
    | _ -> failwith "Airframe.Include.from_xml: unreachable"

end

module Target = struct

  type t = { name: string;
             board: string;
             modules: Module.config list;
             autopilot: Autopilot.t option;
             configures: Module.configure list;
             defines: Module.define list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("target", attrs, children) as xml ->
        { name = List.assoc "name" attrs;
          board = List.assoc "board" attrs;
          modules = ExtXml.parse_children "module" Module.config_from_xml children;
          autopilot = begin try Some (Autopilot.from_xml (ExtXml.child xml "autopilot")) with _ -> None end;
          configures = ExtXml.parse_children_attribs "configure" Module.parse_configure children;
          defines = ExtXml.parse_children_attribs "define" Module.parse_define children;
          xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Firmware = struct

  type t = { name: string;
             targets: Target.t list;
             modules: Module.config list;
             autopilot: Autopilot.t option;
             configures: Module.configure list;
             defines: Module.define list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("firmware", [("name", name)], children) as xml ->
        { name;
          targets = ExtXml.parse_children "targets" Target.from_xml children;
          modules = ExtXml.parse_children "module" Module.config_from_xml children;
          autopilot = begin try Some (Autopilot.from_xml (ExtXml.child xml "autopilot")) with _ -> None end;
          configures = ExtXml.parse_children_attribs "configure" Module.parse_configure children;
          defines = ExtXml.parse_children_attribs "define" Module.parse_define children;
          xml }
    | _ -> failwith "Airframe.Firmware.from_xml: unreachable"

end

type t = {
  filename: string;
  name: string;
  includes: Include.t list;
  modules: Module.config list;
  firmwares: Firmware.t list;
  autopilots: Autopilot.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("airframe", [("name", name)], children) as xml ->
      { filename = ""; name;
        includes = ExtXml.parse_children "include" Include.from_xml children;
        modules = ExtXml.parse_children "modules" Module.config_from_xml children;
        firmwares = ExtXml.parse_children "firmware" Firmware.from_xml children;
        autopilots = ExtXml.parse_children "autopilot" Autopilot.from_xml children;
        xml }
  | _ -> failwith "Airframe.from_xml: unreachable"

let from_file = fun filename ->
  let af = from_xml (Xml.parse_file filename) in
  { af with filename }
