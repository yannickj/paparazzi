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
 * Settings module for parsing XML config files
 *)

let get_opt attrib attribs = try Some (List.assoc attrib attribs) with _ -> None


module Dl_setting = struct

  type t = {
    var: string;
    shortname: string option;
    handler: string option;
    header: string option;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("dl_setting", attribs, _) as xml ->
        let test attrib = List.mem_assoc attrib attribs in
        {
          var = List.assoc "var" attribs;
          shortname = get_opt "shortname" attribs;
          handler = get_opt "handler" attribs;
          header = begin
            match test "module", test "header" with
            | true, _ ->
                Printf.eprintf "Warning: please rename 'module' attribute in settings with 'header'";
                Some (List.assoc "module" attribs)
            | false, true -> Some (List.assoc "header" attribs)
            | false, false -> None
          end;
          xml;
        }
    | _ -> failwith "Settings.Dl_setting.from_xml: unreachable"

end

module Dl_settings = struct

  type t = {
    name: string option;
    dl_settings: t list;
    dl_setting: Dl_setting.t list;
    headers: string list;
    xml: Xml.xml; }

  let rec iter_xml s = function
    | Xml.Element ("dl_settings", attribs, children) as xml ->
        {
          name = get_opt "name" attribs;
          dl_settings = List.map (iter_xml s) children;
          dl_setting = [];
          headers = [];
          xml;
        }
    | Xml.Element ("dl_setting", attribs, _) as xml ->
        { s with dl_setting = Dl_setting.from_xml xml :: s.dl_setting }
    | Xml.Element ("include", [("header", name)], _) ->
        { s with headers = name :: s.headers }
    | _ -> failwith "Settings.Dl_settings.iter_xml: unreachable"

  let from_xml = iter_xml { name = None; dl_settings = []; dl_setting = []; headers = []; xml = Xml.Element ("dl_settings", [], []) }

end

type t = {
  name: string option; (* for modules' settings *)
  target: string option;
  dl_settings: Dl_settings.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("settings", attribs, children) as xml ->
      {
        name = get_opt "name" attribs;
        target = get_opt "target" attribs;
        dl_settings = List.map Dl_settings.from_xml children;
        xml;
      }
  | _ -> failwith "Settings.from_xml: unreachable"

(**
 *  Get singletonized list of headers from a Settings.t
 *)
let get_headers = fun settings ->
  let rec iter = fun headers s ->
    let headers = List.fold_left (fun hl dl_s ->
      match dl_s.Dl_setting.header with
      | Some x -> if List.mem x hl then hl else x :: hl
      | None -> hl
    ) headers s.Dl_settings.dl_setting in
    let headers = List.fold_left (fun hl h ->
      if List.mem h hl then hl else h :: hl
    ) headers s.Dl_settings.headers in
    let headers = List.fold_left (fun hl dl_ss ->
      iter hl dl_ss
    ) headers s.Dl_settings.dl_settings in
    headers
  in
  List.fold_left iter [] settings.dl_settings


