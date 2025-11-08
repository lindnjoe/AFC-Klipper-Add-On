# Filament Group
#
# Copyright (C) 2023-2025 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from typing import List, Tuple, Any, Optional

class FilamentGroup:
    """
    Defines a group of filament spools that can be used interchangeably.
    
    A filament group (e.g., "T0", "T1") represents a collection of spool bays
    that contain the same type of filament. When one spool runs out, the system
    can automatically switch to another spool in the same group.
    
    Key Attributes:
    - group_name: Name of the group (e.g., "T0", "T1", "T2")
    - bays: List of (oams_object, bay_index) tuples representing available spools
    - oams: List of unique OAMS units referenced by this group
    
    Configuration Format:
    [filament_group T0]
    group = "oams1-0", "oams1-1", "oams2-0"
    
    This creates a group "T0" with spools from:
    - OAMS1 bay 0
    - OAMS1 bay 1  
    - OAMS2 bay 0
    """
    
    def __init__(self, config):
        # Core printer interface
        self.printer = config.get_printer()
        self.group_name: str = config.get_name().split()[-1]  # Extract group name (e.g., "T0")
        
        # Storage for bay assignments
        self.bays: List[Tuple[Any, int]] = []  # List of (oams_object, bay_index) tuples
        self.oams: List[Any] = []  # List of unique OAMS objects referenced by this group
        
        # Initialize bay assignments from config
        self._initialize_bays(config)

    def _initialize_bays(self, config) -> None:
        """
        Parse bay assignments from configuration and initialize bay list.
        
        Expected format: group = "oams1-0", "oams1-1", "oams2-0"
        Each entry is "oams_name-bay_index" where bay_index is 0-3.
        """
        bay_assignments = config.get("group").split(",")
        
        for bay_assignment in bay_assignments:
            # Remove quotes and whitespace, then split on hyphen
            clean_assignment = bay_assignment.strip().strip('"')
            
            try:
                oams_name, bay_index_str = clean_assignment.split("-")
                bay_index = int(bay_index_str)
                
                # Validate bay index
                if bay_index < 0 or bay_index > 3:
                    raise ValueError(f"Bay index {bay_index} out of range (0-3)")
                
                # Look up OAMS object
                oam = self.printer.lookup_object("oams " + oams_name.strip())
                if oam is None:
                    raise ValueError(f"OAMS object '{oams_name}' not found")
                
                # Add bay to group
                self.add_bay(oam, bay_index)
                
            except ValueError as e:
                logging.error(f"FilamentGroup {self.group_name}: Invalid bay assignment '{clean_assignment}': {e}")
                raise
            except Exception as e:
                logging.error(f"FilamentGroup {self.group_name}: Error parsing bay assignment '{clean_assignment}': {e}")
                raise

    def add_bay(self, oam: Any, bay_index: int) -> None:
        """
        Add a spool bay to this filament group.
        
        Args:
            oam: OAMS object reference
            bay_index: Bay index (0-3) on the OAMS unit
        """
        bay_id = (oam, bay_index)
        self.bays.append(bay_id)
        
        # Track unique OAMS units
        if oam not in self.oams:
            self.oams.append(oam)
            
        logging.info(f"FilamentGroup {self.group_name}: Added bay {bay_index} from OAMS {oam.name}")

    def is_any_spool_loaded(self) -> bool:
        """
        Check if any spool in this group is currently loaded.
        
        Returns:
            True if at least one spool in the group is loaded
        """
        for oam, bay_index in self.bays:
            if oam.is_bay_loaded(bay_index):
                return True
        return False
    
    def get_available_spools(self) -> List[Tuple[Any, int]]:
        """
        Get list of spools in this group that are ready to load.
        
        Returns:
            List of (oams_object, bay_index) tuples for spools that have filament ready
        """
        available = []
        for oam, bay_index in self.bays:
            if oam.is_bay_ready(bay_index) and not oam.is_bay_loaded(bay_index):
                available.append((oam, bay_index))
        return available
    
    def get_loaded_spool(self) -> Optional[Tuple[Any, int]]:
        """
        Get the currently loaded spool from this group.
        
        Returns:
            (oams_object, bay_index) tuple for loaded spool, or None if no spool loaded
        """
        for oam, bay_index in self.bays:
            if oam.is_bay_loaded(bay_index):
                return (oam, bay_index)
        return None
    
    def get_next_available_spool(self) -> Optional[Tuple[Any, int]]:
        """
        Get the next available spool for automatic loading.
        
        Returns:
            (oams_object, bay_index) tuple for next spool to load, or None if none available
        """
        available = self.get_available_spools()
        return available[0] if available else None

    def get_status(self, eventtime = None) -> dict:
        """
        Get current status of this filament group.
        
        Returns:
            Dictionary with group status information
        """
        loaded_spool = self.get_loaded_spool()
        available_spools = self.get_available_spools()
        
        return {
            "group_name": self.group_name,
            "total_bays": len(self.bays),
            "loaded_spool": f"{loaded_spool[0].name}-{loaded_spool[1]}" if loaded_spool else None,
            "available_spools": len(available_spools),
            "is_loaded": loaded_spool is not None,
            "has_available": len(available_spools) > 0
        }
    
    def __str__(self) -> str:
        """String representation of the filament group."""
        bay_descriptions = []
        for oam, bay_index in self.bays:
            status = "loaded" if oam.is_bay_loaded(bay_index) else "ready" if oam.is_bay_ready(bay_index) else "empty"
            bay_descriptions.append(f"{oam.name}-{bay_index}({status})")
        
        return f"FilamentGroup {self.group_name}: [{', '.join(bay_descriptions)}]"
    
def load_config_prefix(config):
    return FilamentGroup(config)

def load_config(config):
    return FilamentGroup(config)
