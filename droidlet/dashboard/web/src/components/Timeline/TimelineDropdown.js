/*
Copyright (c) Facebook, Inc. and its affiliates.
*/

import React from "react";
import { handleSearch } from "./TimelineSearch";
import { timelineTypes } from "./Timeline";

import { makeStyles } from "@material-ui/core/styles";
import Input from "@material-ui/core/Input";
import InputLabel from "@material-ui/core/InputLabel";
import MenuItem from "@material-ui/core/MenuItem";
import FormControl from "@material-ui/core/FormControl";
import ListItemText from "@material-ui/core/ListItemText";
import Select from "@material-ui/core/Select";
import Checkbox from "@material-ui/core/Checkbox";

const useStyles = makeStyles((theme) => ({
  formControl: {
    margin: theme.spacing(1),
    minWidth: 120,
    maxWidth: 300,
  },
}));

const ITEM_HEIGHT = 48;
const ITEM_PADDING_TOP = 8;
const MenuProps = {
  PaperProps: {
    style: {
      maxHeight: ITEM_HEIGHT * 4.5 + ITEM_PADDING_TOP,
      width: 250,
    },
  },
};

export default function TimelineDropdown({ stateManager }) {
  const classes = useStyles();
  const names = timelineTypes;
  const [eventName, setEventName] = React.useState(names);

  const handleChange = (event) => {
    setEventName(event.target.value);
    stateManager.memory.timelineFilters = event.target.value;
    stateManager.updateTimeline();
    // update search results
    handleSearch(stateManager, stateManager.memory.timelineSearchPattern);
  };

  return (
    <div>
      <FormControl className={classes.formControl}>
        <InputLabel id="multiple-checkbox-label">Filters</InputLabel>
        <Select
          labelId="multiple-checkbox-label"
          id="multiple-checkbox"
          multiple
          value={eventName}
          onChange={handleChange}
          input={<Input />}
          renderValue={(selected) => selected.join(", ")}
          MenuProps={MenuProps}
        >
          {names.map((name) => (
            <MenuItem key={name} value={name}>
              <Checkbox checked={eventName.indexOf(name) > -1} />
              <ListItemText primary={name} />
            </MenuItem>
          ))}
        </Select>
      </FormControl>
    </div>
  );
}
