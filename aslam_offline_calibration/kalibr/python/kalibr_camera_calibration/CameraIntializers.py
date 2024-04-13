import sm_python
import aslam_backend_python as aopt
import aslam_cv_python as cv
import numpy as np

def addPoseDesignVariable(problem, T0=sm_python.Transformation()):
    q_Dv = aopt.RotationQuaternionDv( T0.q() )
    q_Dv.setActive( True )
    problem.addDesignVariable(q_Dv)
    t_Dv = aopt.EuclideanPointDv( T0.t() )
    t_Dv.setActive( True )
    problem.addDesignVariable(t_Dv)
    return aopt.TransformationBasicDv( q_Dv.toExpression(), t_Dv.toExpression() )

def stereoCalibrate(camL_geometry, camH_geometry, obslist, distortionActive=False, baseline=None):
    #####################################################
    ## find initial guess as median of  all pnp solutions
    #####################################################
    if baseline is None:
        r=[]; t=[]
        for obsL, obsH in obslist:
            #if we have observations for both camss
            if obsL is not None and obsH is not None:
                success, T_L = camL_geometry.geometry.estimateTransformation(obsL)
                success, T_H = camH_geometry.geometry.estimateTransformation(obsH)
                
                baseline = T_H.inverse()*T_L
                t.append(baseline.t())
                rv=sm_python.RotationVector()
                r.append(rv.rotationMatrixToParameters( baseline.C() ))
        
        r_median = np.median(np.asmatrix(r), axis=0).flatten().T
        R_median = rv.parametersToRotationMatrix(r_median)
        t_median = np.median(np.asmatrix(t), axis=0).flatten().T
        
        baseline_HL = sm_python.Transformation( sm_python.rt2Transform(R_median, t_median) )
    else:
        baseline_HL = baseline
    
    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        dL = camL_geometry.geometry.projection().distortion().getParameters().flatten()
        pL = camL_geometry.geometry.projection().getParameters().flatten()
        dH = camH_geometry.geometry.projection().distortion().getParameters().flatten()
        pH = camH_geometry.geometry.projection().getParameters().flatten()
        sm_python.logDebug("initial guess for stereo calib: {0}".format(baseline_HL.T()))
        sm_python.logDebug("initial guess for intrinsics camL: {0}".format(pL))
        sm_python.logDebug("initial guess for intrinsics camH: {0}".format(pH))
        sm_python.logDebug("initial guess for distortion camL: {0}".format(dL))
        sm_python.logDebug("initial guess for distortion camH: {0}".format(dH))    
    
    ############################################
    ## solve the bundle adjustment
    ############################################
    problem = aopt.OptimizationProblem()

    #baseline design variable        
    baseline_dv = addPoseDesignVariable(problem, baseline_HL)
        
    #target pose dv for all target views (=T_camL_w)
    target_pose_dvs = list()
    for obsL, obsH in obslist:
        if obsL is not None: #use camL if we have an obs for this one
            success, T_t_cL = camL_geometry.geometry.estimateTransformation(obsL)
        else:
            success, T_t_cH = camH_geometry.geometry.estimateTransformation(obsH)
            T_t_cL = T_t_cH*baseline_HL #apply baseline for the second camera
            
        target_pose_dv = addPoseDesignVariable(problem, T_t_cL)
        target_pose_dvs.append(target_pose_dv)
    
    #add camera dvs
    camL_geometry.setDvActiveStatus(True, distortionActive, False)
    camH_geometry.setDvActiveStatus(True, distortionActive, False)
    problem.addDesignVariable(camL_geometry.dv.distortionDesignVariable())
    problem.addDesignVariable(camL_geometry.dv.projectionDesignVariable())
    problem.addDesignVariable(camL_geometry.dv.shutterDesignVariable())
    problem.addDesignVariable(camH_geometry.dv.distortionDesignVariable())
    problem.addDesignVariable(camH_geometry.dv.projectionDesignVariable())
    problem.addDesignVariable(camH_geometry.dv.shutterDesignVariable())
    
    ############################################
    ## add error terms
    ############################################
    
    #corner uncertainty
    # \todo pass in the detector uncertainty somehow.
    cornerUncertainty = 1.0
    R = np.eye(2) * cornerUncertainty * cornerUncertainty
    invR = np.linalg.inv(R)
        
    #Add reprojection error terms for both cameras
    reprojectionErrors0 = []; reprojectionErrors1 = []
            
    for cidx, cam in enumerate([camL_geometry, camH_geometry]):
        sm_python.logDebug("stereoCalibration: adding camera error terms for {0} calibration targets".format(len(obslist)))

        #get the image and target points corresponding to the frame
        target = cam.ctarget.detector.target()
        
        #add error terms for all observations
        for view_id, obstuple in enumerate(obslist):
            
            #add error terms if we have an observation for this cam
            obs=obstuple[cidx]
            if obs is not None:
                T_cam_w = target_pose_dvs[view_id].toExpression().inverse()
            
                #add the baseline for the second camera
                if cidx!=0:
                    T_cam_w =  baseline_dv.toExpression() * T_cam_w
                    
                for i in range(0, target.size()):
                    p_target = aopt.HomogeneousExpression(sm_python.toHomogeneous(target.point(i)));
                    valid, y = obs.imagePoint(i)
                    if valid:
                        # Create an error term.
                        rerr = cam.model.reprojectionError(y, invR, T_cam_w * p_target, cam.dv)
                        rerr.idx = i
                        problem.addErrorTerm(rerr)
                    
                        if cidx==0:
                            reprojectionErrors0.append(rerr)
                        else:
                            reprojectionErrors1.append(rerr)
                                                        
        sm_python.logDebug("stereoCalibrate: added {0} camera error terms".format( len(reprojectionErrors0)+len(reprojectionErrors1) ))
        
    ############################################
    ## solve
    ############################################       
    options = aopt.Optimizer2Options()
    options.verbose = True if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug else False
    options.nThreads = 4
    options.convergenceDeltaX = 1e-3
    options.convergenceDeltaJ = 1
    options.maxIterations = 200
    options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(10)

    optimizer = aopt.Optimizer2(options)
    optimizer.setProblem(problem)

    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        sm_python.logDebug("Before optimization:")
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors0 ])
        sm_python.logDebug( " Reprojection error squarred (camL):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors1 ])
        sm_python.logDebug( " Reprojection error squarred (camH):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
    
        sm_python.logDebug("baseline={0}".format(baseline_dv.toTransformationMatrix()))
    
    try: 
        retval = optimizer.optimize()
        if retval.linearSolverFailure:
            sm_python.logError("stereoCalibrate: Optimization failed!")
        success = not retval.linearSolverFailure
    except:
        sm_python.logError("stereoCalibrate: Optimization failed!")
        success = False
    
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        sm_python.logDebug("After optimization:")
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors0 ])
        sm_python.logDebug( " Reprojection error squarred (camL):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors1 ])
        sm_python.logDebug( " Reprojection error squarred (camH):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
    
    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        dL = camL_geometry.geometry.projection().distortion().getParameters().flatten()
        pL = camL_geometry.geometry.projection().getParameters().flatten()
        dH = camH_geometry.geometry.projection().distortion().getParameters().flatten()
        pH = camH_geometry.geometry.projection().getParameters().flatten()
        sm_python.logDebug("guess for intrinsics camL: {0}".format(pL))
        sm_python.logDebug("guess for intrinsics camH: {0}".format(pH))
        sm_python.logDebug("guess for distortion camL: {0}".format(dL))
        sm_python.logDebug("guess for distortion camH: {0}".format(dH))    
    
    if success:
        baseline_HL = sm_python.Transformation(baseline_dv.toTransformationMatrix())
        return success, baseline_HL
    else:
        #return the intiial guess if we fail
        return success, baseline_HL


def calibrateIntrinsics(cam_geometry, obslist, distortionActive=True, intrinsicsActive=True):
    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        d = cam_geometry.geometry.projection().distortion().getParameters().flatten()
        p = cam_geometry.geometry.projection().getParameters().flatten()
        sm_python.logDebug("calibrateIntrinsics: intrinsics guess: {0}".format(p))
        sm_python.logDebug("calibrateIntrinsics: distortion guess: {0}".format(d))
    
    ############################################
    ## solve the bundle adjustment
    ############################################
    problem = aopt.OptimizationProblem()
    
    #add camera dvs
    cam_geometry.setDvActiveStatus(intrinsicsActive, distortionActive, False)
    problem.addDesignVariable(cam_geometry.dv.distortionDesignVariable())
    problem.addDesignVariable(cam_geometry.dv.projectionDesignVariable())
    problem.addDesignVariable(cam_geometry.dv.shutterDesignVariable())
    
    #corner uncertainty
    cornerUncertainty = 1.0
    R = np.eye(2) * cornerUncertainty * cornerUncertainty
    invR = np.linalg.inv(R)
    
    #get the image and target points corresponding to the frame
    target = cam_geometry.ctarget.detector.target()
    
    #target pose dv for all target views (=T_camL_w)
    reprojectionErrors = [];    
    sm_python.logDebug("calibrateIntrinsics: adding camera error terms for {0} calibration targets".format(len(obslist)))
    target_pose_dvs=list()
    for obs in obslist: 
        success, T_t_c = cam_geometry.geometry.estimateTransformation(obs)
        target_pose_dv = addPoseDesignVariable(problem, T_t_c)
        target_pose_dvs.append(target_pose_dv)
        
        T_cam_w = target_pose_dv.toExpression().inverse()
    
        ## add error terms
        for i in range(0, target.size()):
            p_target = aopt.HomogeneousExpression(sm_python.toHomogeneous(target.point(i)));
            valid, y = obs.imagePoint(i)
            if valid:
                rerr = cam_geometry.model.reprojectionError(y, invR, T_cam_w * p_target, cam_geometry.dv)
                problem.addErrorTerm(rerr)
                reprojectionErrors.append(rerr)
                                                    
    sm_python.logDebug("calibrateIntrinsics: added {0} camera error terms".format(len(reprojectionErrors)))
    
    ############################################
    ## solve
    ############################################       
    options = aopt.Optimizer2Options()
    options.verbose = True if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug else False
    options.nThreads = 4
    options.convergenceDeltaX = 1e-3
    options.convergenceDeltaJ = 1
    options.maxIterations = 200
    options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(10)

    optimizer = aopt.Optimizer2(options)
    optimizer.setProblem(problem)

    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        sm_python.logDebug("Before optimization:")
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors ])
        sm_python.logDebug( " Reprojection error squarred (camL):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
    
    #run intrinsic calibration
    try: 
        retval = optimizer.optimize()
        if retval.linearSolverFailure:
            sm_python.logError("calibrateIntrinsics: Optimization failed!")
        success = not retval.linearSolverFailure

    except:
        sm_python.logError("calibrateIntrinsics: Optimization failed!")
        success = False
    
    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        d = cam_geometry.geometry.projection().distortion().getParameters().flatten()
        p = cam_geometry.geometry.projection().getParameters().flatten()
        sm_python.logDebug("calibrateIntrinsics: guess for intrinsics cam: {0}".format(p))
        sm_python.logDebug("calibrateIntrinsics: guess for distortion cam: {0}".format(d))
    
    return success


def solveFullBatch(cameras, baseline_guesses, graph):    
    ############################################
    ## solve the bundle adjustment
    ############################################
    problem = aopt.OptimizationProblem()
    
    #add camera dvs
    for cam in cameras:
        cam.setDvActiveStatus(True, True, False)
        problem.addDesignVariable(cam.dv.distortionDesignVariable())
        problem.addDesignVariable(cam.dv.projectionDesignVariable())
        problem.addDesignVariable(cam.dv.shutterDesignVariable())
    
    baseline_dvs = list()
    for baseline_idx in range(0, len(cameras)-1): 
        baseline_dv = aopt.TransformationDv(baseline_guesses[baseline_idx])
        
        for i in range(0, baseline_dv.numDesignVariables()):
            problem.addDesignVariable(baseline_dv.getDesignVariable(i))
        
        baseline_dvs.append( baseline_dv )
    
    #corner uncertainty
    cornerUncertainty = 1.0
    R = np.eye(2) * cornerUncertainty * cornerUncertainty
    invR = np.linalg.inv(R)
    
    #get the target
    target = cameras[0].ctarget.detector.target()

    #Add calibration target reprojection error terms for all camera in chain
    target_pose_dvs = list()
      
    #shuffle the views
    reprojectionErrors = [];    
    timestamps = graph.obs_db.getAllViewTimestamps()
    for view_id, timestamp in enumerate(timestamps):
        
        #get all observations for all cams at this time
        obs_tuple = graph.obs_db.getAllObsAtTimestamp(timestamp)

        #create a target pose dv for all target views (= T_cam0_w)
        T0 = graph.getTargetPoseGuess(timestamp, cameras, baseline_guesses)
        target_pose_dv = addPoseDesignVariable(problem, T0)
        target_pose_dvs.append(target_pose_dv)
        

        for cidx, obs in obs_tuple:
            cam = cameras[cidx]
              
            #calibration target coords to camera X coords
            T_cam0_calib = target_pose_dv.toExpression().inverse()

            #build pose chain (target->cam0->baselines->camN)
            T_camN_calib = T_cam0_calib
            for idx in range(0, cidx):
                T_camN_calib = baseline_dvs[idx].toExpression() * T_camN_calib
                
        
            ## add error terms
            for i in range(0, target.size()):
                p_target = aopt.HomogeneousExpression(sm_python.toHomogeneous(target.point(i)));
                valid, y = obs.imagePoint(i)
                if valid:
                    rerr = cameras[cidx].model.reprojectionError(y, invR, T_camN_calib * p_target, cameras[cidx].dv)
                    problem.addErrorTerm(rerr)
                    reprojectionErrors.append(rerr)
                                                    
    sm_python.logDebug("solveFullBatch: added {0} camera error terms".format(len(reprojectionErrors)))
    
    ############################################
    ## solve
    ############################################       
    options = aopt.Optimizer2Options()
    options.verbose = True if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug else False
    options.nThreads = 4
    options.convergenceDeltaX = 1e-3
    options.convergenceDeltaJ = 1
    options.maxIterations = 250
    options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(10)

    optimizer = aopt.Optimizer2(options)
    optimizer.setProblem(problem)

    #verbose output
    if sm_python.getLoggingLevel()==sm_python.LoggingLevel.Debug:
        sm_python.logDebug("Before optimization:")
        e2 = np.array([ e.evaluateError() for e in reprojectionErrors ])
        sm_python.logDebug( " Reprojection error squarred (camL):  mean {0}, median {1}, std: {2}".format(np.mean(e2), np.median(e2), np.std(e2) ) )
    
    #run intrinsic calibration
    try:
        retval = optimizer.optimize()
        if retval.linearSolverFailure:
            sm_python.logError("calibrateIntrinsics: Optimization failed!")
        success = not retval.linearSolverFailure

    except:
        sm_python.logError("calibrateIntrinsics: Optimization failed!")
        success = False

    baselines=list()
    for baseline_dv in baseline_dvs:
        baselines.append( sm_python.Transformation(baseline_dv.T()) )
    
    return success, baselines

