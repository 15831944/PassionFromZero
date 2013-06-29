load 'commonTaskParams.rb'


#Input Interfaces
ReadOnlyInterfaces = {
    'TransformSource' => Interfaces.get('TransformSourceConfigFile'),
    'VehicleState' => Interfaces.get('VehicleStateInterpolated'),
    'RoadWorldModelInput' => Interfaces.get('RoadWorldModelInputFile'),
    'IBEOScanInput' => Interfaces.get('IBEOScanInput')
}


#Output Interfaces
WriteOnlyInterfaces = {
    'SegmentationMapOutput' => Interfaces.get('SegmentationMapOutput'),
}


#Parameters
Parameters = CommonTaskParams::Parameters.dup;
Parameters.update( {
                    'loggerThreshold' => 'info',
                    'cycleRate_hz' => 10,
                    'Max Messages' => 10,
                    'mapSize_m' => 100,
                    'cellSize_m' => 0.25, 
                    'hysteresis_s' => 1.5,

                    'EnablePublishSegmentationMap' => true,
                    'CCSWay' => 8,                # 4: 4-way; 8: 8-way
                    'CellOccupiedThreshold' => 2, # > means occupied, < means unoccupied
                    
                    'Debug_ScanPoints' => true,

                 } )


#???
if (Utility.WhichRobot != "srx")
    ReadOnlyInterfaces.update(  { "Clock" => Interfaces::get("ClockPlaybackClient") }  )
end


#Execution
Execution = {
    'executableName' => 'Segmentation'
}








