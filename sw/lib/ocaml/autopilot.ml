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
 * Autopilot module for parsing XML config files
 *)

module OT = Ocaml_tools

let parse_children_attribs = fun tag f children ->
  List.fold_left (fun l x -> if Xml.tag x = tag then f (Xml.attribs x) :: l else l)
  [] children

module Module_fp = struct

  type t = { name: string;
             mtype: string option;
             configures: Module.configure list;
             defines: Module.define list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("module", attrs, children) as xml ->
        { name = List.assoc "name" attrs;
          mtype = OT.assoc_opt "type" attrs;
          configures = parse_children_attribs "configure" Module.parse_configure children;
          defines = parse_children_attribs "define" Module.parse_define children;
          xml }
    | _ -> failwith "Airframe.Module_af.from_xml: unreachable"

end

type t = {
  modules: Module_fp.t list;
  xml: Xml.xml;
}

let from_xml = function
  | Xml.Element ("autopilot", _, children) as xml ->
      let modules = List.fold_left (fun m el ->
        if Xml.tag el = "modules" then
          m @ List.map Module_fp.from_xml (Xml.children el)
        else
          m
      ) [] children in
      { modules; xml }
  | _ -> failwith "Autopilot.from_xml: unreachable"

