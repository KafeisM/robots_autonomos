--[[

Hokuyo URG 04LX UG01 Fast Lidar

Copyright (C) 2023 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

--]]


function sysCall_init()
    maxScanDistance=5
    showLines=true
    generateData=true
    rangeData=true -- if false, then X/Y/Z data rel to sensor base
    discardMaxDistPts=true

    self=sim.getObject('.')
    visionSensors={sim.getObject("./sensor1"),sim.getObject("./sensor2")}
    local collection=sim.createCollection(0)
    sim.addItemToCollection(collection,sim.handle_all,-1,0)
    sim.addItemToCollection(collection,sim.handle_tree,self,1)
    sim.setObjectInt32Param(visionSensors[1],sim.visionintparam_entity_to_render,collection)
    sim.setObjectInt32Param(visionSensors[2],sim.visionintparam_entity_to_render,collection)
    sim.setObjectFloatParam(visionSensors[1],sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParam(visionSensors[2],sim.visionfloatparam_far_clipping,maxScanDistance)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,10000,red)
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines)
end 

function sysCall_sensing() 
    local measuredData={}
    
    sim.addDrawingObjectItem(lines,nil)
    for i=1,2,1 do
        local r,t,u=sim.readVisionSensor(visionSensors[i])
        if u then
            local sensorM=sim.getObjectMatrix(visionSensors[i],sim.handle_world)
            local relRefM=sim.getObjectMatrix(self,sim.handle_world)
            relRefM=sim.getMatrixInverse(relRefM)
            relRefM=sim.multiplyMatrices(relRefM,sensorM)
            local p={0,0,0}
            p=sim.multiplyVector(sensorM,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u[2]-1,1 do
                for k=0,u[1]-1,1 do
                    local w=2+4*(j*u[1]+k)
                    local v={u[w+1],u[w+2],u[w+3],u[w+4]}
                    if generateData then
                        if rangeData then
                            table.insert(measuredData,v[4])
                        else
                            if v[4]<maxScanDistance*0.9999 or not discardMaxDistPts then
                                p=sim.multiplyVector(relRefM,v)
                                table.insert(measuredData,p[1])
                                table.insert(measuredData,p[2])
                                table.insert(measuredData,p[3])
                            end
                        end
                    end
                    if showLines then
                        p=sim.multiplyVector(sensorM,v)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
    end
    packedData=sim.packFloatTable(measuredData)
    if not packedData then
        packedData=""
    end
    sim.setStringSignal("PioneerP3dxLidarData",packedData)
end 
