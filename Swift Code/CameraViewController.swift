import UIKit
import AVFoundation
import Photos
import VideoToolbox
import Foundation
import AVKit
import CoreVideo
import CoreImage
import CoreMotion
import MobileCoreServices
import Accelerate
import Photos  // luozc
import FirebaseCore
import Firebase
import FirebaseFirestore
import FirebaseDatabase
import FirebaseAuth
import Compression
import FirebaseStorage
import ImageCaptureCore
import SwiftUI
import SceneKit
import GameController

func convertDepthData(depthMap: CVPixelBuffer) -> Data {
    let width = CVPixelBufferGetWidth(depthMap)
    let height = CVPixelBufferGetHeight(depthMap)
    var convertedDepthMap: [[Float]] = Array(
        repeating: Array(repeating: 0, count: width),
        count: height
    )
    CVPixelBufferLockBaseAddress(depthMap, CVPixelBufferLockFlags(rawValue: 2))
    let floatBuffer = unsafeBitCast(
        CVPixelBufferGetBaseAddress(depthMap),
        to: UnsafeMutablePointer<Float>.self
    )
//MARK: Converting Float to Int
    for row in 0 ..< height {
        for col in 0 ..< width {
            convertedDepthMap[row][col] = round(Float(floatBuffer[width * row + col])/10) // Divide by /10 to get centimeters round(Float(floatBuffer[width * row + col])/10)
            //TODO: Converted to cm by dividing the depth values by factor of 10
        }
    }
    CVPixelBufferUnlockBaseAddress(depthMap, CVPixelBufferLockFlags(rawValue: 2))
    
    var jsonDict: [String : Any]
    jsonDict = [
            "depth_data" : convertedDepthMap
    ]

    let jsonStringData = try! JSONSerialization.data(
        withJSONObject: jsonDict,
        options: .prettyPrinted
    )
    return jsonStringData
}
//MARK: Motion Structure
struct SensorData {
    var roll: [Double] = []
    var pitch: [Double] = []
    var yaw: [Double] = []
    var rotationRateX: [Double] = []
    var rotationRateY: [Double] = []
    var rotationRateZ: [Double] = []
    var userAccelerationX: [Double] = []
    var userAccelerationY: [Double] = []
    var userAccelerationZ: [Double] = []
    var normalizedAccX: [Double] = []
    var normalizedAccY: [Double] = []
    var normalizedAccZ: [Double] = []
    var heading: [Double] = []
    var magneticX: [Double] = []
    var magneticY: [Double] = []
    var magneticZ: [Double] = []
    var quaternionX: [Double] = []
    var quaternionY: [Double] = []
    var quaternionZ: [Double] = []
    var quaternionW: [Double] = []
}

func sensorToJSON(sensorList: [String]) -> Data {
    var jsonDict: [String : Any]
    jsonDict = [
            "spatial_data" : sensorList
    ]

    let jsonStringData = try! JSONSerialization.data(
        withJSONObject: jsonDict,
        options: .prettyPrinted
    )
    return jsonStringData
}

//MARK: MAIN Function
class CameraViewController: UIViewController, AVCapturePhotoCaptureDelegate {
//  NOTE: Database Reference
    var ref: DatabaseReference!
    var timer = Timer()
//  NOTE: CMMotionManager
    var count: Int!
    var data: SensorData! = SensorData()
    var lineList = [String]()
    var motionManager: CMMotionManager? = CMMotionManager()
    
    var videoPreviewLayer: AVCaptureVideoPreviewLayer?
    var StateDeterminator = false
    
    @IBOutlet weak var cameraView: UIView!
    
//  MARK: Add button
    @IBOutlet var autoSavingSwitch: UISwitch!
    private var savingEnabled = false
    
    
    let session: AVCaptureSession = AVCaptureSession()
    let photoOutput = AVCapturePhotoOutput()
    private let videoDeviceDiscoverySession = AVCaptureDevice.DiscoverySession(
        deviceTypes: [.builtInTrueDepthCamera],
        mediaType: .video,
        position: .front)
    
    private let sessionQueue = DispatchQueue(label: "SessionQueue", attributes: [], autoreleaseFrequency: .workItem)
    private let processingQueue = DispatchQueue(label: "photo processing queue", attributes: [], autoreleaseFrequency: .workItem)
    
    private let photoDepthConverter = DepthToGrayscaleConverter()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        UIApplication.shared.isIdleTimerDisabled = true
        
        ref = Database.database(url: "https://depthstreamer-default-rtdb.europe-west1.firebasedatabase.app").reference()
        
        let storage = Storage.storage(url:"gs://depthstreamer.appspot.com")
        let storageRef = storage.reference()
        
        /* setup input and output */
        let videoDevice: AVCaptureDevice? = videoDeviceDiscoverySession.devices.first
        guard
            let videoDeviceInput = try? AVCaptureDeviceInput(device: videoDevice!), session.canAddInput(videoDeviceInput)
            else {
                print("Cannot addd input")
                return
        }
        
        session.beginConfiguration()
        
        session.addInput(videoDeviceInput)
        
        guard session.canAddOutput(photoOutput) else {
            print("cannot add photo output")
            return
        }
        session.addOutput(photoOutput)
//      session.sessionPreset = .photo
        session.sessionPreset = AVCaptureSession.Preset.vga640x480
        
        session.commitConfiguration()
        
        photoOutput.isDepthDataDeliveryEnabled = photoOutput.isDepthDataDeliverySupported
        
        
        videoPreviewLayer = AVCaptureVideoPreviewLayer(session: session)
        videoPreviewLayer?.frame = cameraView.layer.bounds
        cameraView.layer.addSublayer(videoPreviewLayer!)
        
        session.startRunning()

        ref.child("capture").child("captureState").observe(.value, with: { snapshot in
            self.StateDeterminator = snapshot.value as! Bool
        })
        count = 0
    }
//  MARK: Reset Sensor Data
    func resetSensorData() {
            count = 0
            data.roll.removeAll()
            data.pitch.removeAll()
            data.yaw.removeAll()
            data.rotationRateX.removeAll()
            data.rotationRateY.removeAll()
            data.rotationRateZ.removeAll()
            data.userAccelerationX.removeAll()
            data.userAccelerationY.removeAll()
            data.userAccelerationZ.removeAll()
            data.normalizedAccX.removeAll()
            data.normalizedAccY.removeAll()
            data.normalizedAccZ.removeAll()
            data.heading.removeAll()
            data.magneticX.removeAll()
            data.magneticY.removeAll()
            data.magneticZ.removeAll()
            data.quaternionX.removeAll()
            data.quaternionY.removeAll()
            data.quaternionZ.removeAll()
            data.quaternionW.removeAll()
        }
//  MARK: Reset Sensor Buffer List
    func resetLineData(){
        lineList.removeAll()
    }
    
//  MARK: Write Sensor Data To List
    func writeSensorData() {
                for i in 0 ..< count {
                    let msg1 = String(format:"%.20g,%.20g,%.20g",
                                        data.roll[i],
                                        data.pitch[i],
                                        data.yaw[i])
                    let msg2 = String(format:"%.20g,%.20g,%.20g",
                                        data.rotationRateX[i],
                                        data.rotationRateY[i],
                                        data.rotationRateZ[i])
                    let msg3 = String(format:"%.20g,%.20g,%.20g",
                                        data.userAccelerationX[i],
                                        data.userAccelerationY[i],
                                        data.userAccelerationZ[i])
                    let msg4 = String(format:"%.20g,%.20g,%.20g,%.20g",
                                        data.quaternionX[i],
                                        data.quaternionY[i],
                                        data.quaternionZ[i],
                                        data.quaternionW[i])
                    let msg5 = String(format:"%.20g,%.20g,%.20g",
                                        data.magneticX[i],
                                        data.magneticY[i],
                                        data.magneticZ[i])
                    let msg6 = String(format:"%.20g,%.20g,%.20g",
                                     data.normalizedAccX[i],
                                     data.normalizedAccY[i],
                                     data.normalizedAccZ[i])
                    
                    let line = msg1 + "," + msg2 + "," + msg3 + "," + msg4 + "," + msg5 + "," + msg6
                    print(line)
                    lineList.append(line)
        }
    }
    
    
//    MARK: Start Motion Updates
    func startQueuedUpdates() {
            guard let motion = motionManager, motion.isDeviceMotionAvailable else {
                // Device motion NOT available
                print("Device motion NOT available")
                return
            }
//        let pi = 3.14159265359
            
        motion.deviceMotionUpdateInterval = 1.0 / 100.0
        motion.showsDeviceMovementDisplay = true
        motion.accelerometerUpdateInterval = 1.0 / 100.0
        motion.startAccelerometerUpdates()
            motion.startDeviceMotionUpdates(
            using: .xMagneticNorthZVertical, // Get the attitude relative to the magnetic north reference frame.
                to: .main, withHandler: { (data, error) in // TODO: main queue not recommended, self.queue
                    // Make sure the data is valid before accessing it.
                    if let validData = data {
                        self.data.roll.append(validData.attitude.roll)
                        self.data.pitch.append(validData.attitude.pitch)
                        self.data.yaw.append(validData.attitude.yaw)

                        self.data.rotationRateX.append(validData.rotationRate.x)
                        self.data.rotationRateY.append(validData.rotationRate.y)
                        self.data.rotationRateZ.append(validData.rotationRate.z)

                        let x = (motion.accelerometerData?.acceleration.x)!
                        let y = (motion.accelerometerData?.acceleration.y)!
                        let z = (motion.accelerometerData?.acceleration.z)!
                        self.data.userAccelerationX.append(x)
                        self.data.userAccelerationY.append(y)
                        self.data.userAccelerationZ.append(z)
                            
                        print("X: \(x), Y: \(y), X: \(z)")

                        self.data.normalizedAccX.append(validData.userAcceleration.x)
                        self.data.normalizedAccY.append(validData.userAcceleration.y)
                        self.data.normalizedAccZ.append(validData.userAcceleration.z)

                        self.data.quaternionX.append(validData.attitude.quaternion.x)
                        self.data.quaternionY.append(validData.attitude.quaternion.y)
                        self.data.quaternionZ.append(validData.attitude.quaternion.z)
                        self.data.quaternionW.append(validData.attitude.quaternion.w)
                        
                        self.data.magneticX.append(validData.magneticField.field.x)
                        self.data.magneticY.append(validData.magneticField.field.y)
                        self.data.magneticZ.append(validData.magneticField.field.z)

//                        print("attitude.pitch: %.5f, attitude.roll: %.5f, attitude.yaw: %.5f", (validData.attitude.pitch * 180 / pi), (validData.attitude.roll * 180 / pi), (validData.attitude.yaw * 180 / pi))
                        
//                        print("magx: \(validData.magneticField.field.x), magx: \(validData.magneticField.field.y),magx: \(validData.magneticField.field.z)")

                        self.data.heading.append(validData.heading)

                        self.count = self.count + 1
                        print(self.count!)
                    }
            })
        
        }
    
//    MARK: Stop Motion Updates
    func stopQueueUpdates() {
            guard let motion = motionManager, motion.isDeviceMotionAvailable else { return }
            motion.stopDeviceMotionUpdates()
            motion.stopAccelerometerUpdates()
    }
    
    @IBAction func didAutoSavingChange(_ sender: UISwitch) {
        processingQueue.async {
            self.savingEnabled = self.autoSavingSwitch.isOn ? true : false
        }
        if self.autoSavingSwitch.isOn{
            print("autoSavingSwitch.isOn == True")
        } else {
            print("autoSavingSwitch.isOn == False")
        }
        
        if StateDeterminator{
            print("StateDeterminator == True")
        }
        else{
            print("StateDeterminator == False")
        }
    }
    
    
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        print("captured")
        print("lenght of accelerometer \(data.userAccelerationY.count), lenght of else \(data.magneticX.count)")
//        processingQueue.sync {
            if let depthData = photo.depthData {
                let depthPixelBuffer = depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32).depthDataMap
                if !self.photoDepthConverter.isPrepared {
                    var depthFormatDescription: CMFormatDescription?
                    CMVideoFormatDescriptionCreateForImageBuffer(allocator: kCFAllocatorDefault,
                                                                 imageBuffer: depthPixelBuffer,
                                                                 formatDescriptionOut: &depthFormatDescription)
                    
                    if let unwrappedDepthFormatDescription = depthFormatDescription {
                        self.photoDepthConverter.prepare(with: unwrappedDepthFormatDescription, outputRetainedBufferCountHint: 3)
                    }
                }
                //MARK: Get GreyScaled Image of the depthmap
                guard let convertedDepthPixelBuffer = self.photoDepthConverter.render(pixelBuffer: depthPixelBuffer) else {
                    print("Unable to convert depth pixel buffer")
                    return
                }
                
//                MARK: Save GrayScaled image to photos
//                let greyImage = UIImage.init(pixelBuffer: convertedDepthPixelBuffer)
//                UIImageWriteToSavedPhotosAlbum(greyImage!, nil, nil, nil)
                
                let depthDataArray = convertDepthData(depthMap: convertedDepthPixelBuffer)
                
                
                self.writeSensorData()
//                print(self.lineList)
                let spatialArray = sensorToJSON(sensorList: self.lineList)
                print(spatialArray)


//            MARK: Device Attitude Information
                
                let formatter = DateFormatter()
                formatter.dateFormat = "dd-MM-yy"
                formatter.locale = Locale.init(identifier: "en_US_POSIX")
                //let timeInterval = Date().timeIntervalSince1970
                //let msecond = CLongLong(round(timeInterval * 1000))
                let dd_MM_YY_FileName = formatter.string(from: Date())
                
                let HH_mm_SS_Child = DateFormatter()
                HH_mm_SS_Child.dateFormat = "HH-mm-ss"
                HH_mm_SS_Child.locale = Locale.init(identifier: "en_US_POSIX")
                let HH_mm_SS_ChildFileName = HH_mm_SS_Child.string(from: Date())
                
                print("***************Entered writing func***************\n")
                    let imageRef = Storage.storage(url: "gs://depthstreamer.appspot.com").reference().child("images/photodata/\(dd_MM_YY_FileName)/\(HH_mm_SS_ChildFileName).heif")
                
                    let depthRef = Storage.storage(url: "gs://depthstreamer.appspot.com").reference().child("images/depthdata/\(dd_MM_YY_FileName)/\(HH_mm_SS_ChildFileName).json")
                
                    let sensorRef = Storage.storage(url: "gs://depthstreamer.appspot.com").reference().child("images/sensordata/\(dd_MM_YY_FileName)/\(HH_mm_SS_ChildFileName)-10.json")
                
                imageRef.putData(photo.fileDataRepresentation()!)
                depthRef.putData(depthDataArray)
                sensorRef.putData(spatialArray)
                print("***************Out writing func***************\n")
            }
//        }
        

        
//      photo.fileDataRepresentation()
        PHPhotoLibrary.requestAuthorization { status in
            guard status == .authorized else { return }
            
//            PHPhotoLibrary.shared().performChanges({
//                let creationRequest = PHAssetCreationRequest.forAsset()
//                creationRequest.addResource(with: .photo, data: photo.fileDataRepresentation()!, options: nil)
//
//            }, completionHandler: nil)
        }
        self.stopQueueUpdates()
        self.resetSensorData()
        self.resetLineData()
        self.startQueuedUpdates()
    }
    func getSettings() -> AVCapturePhotoSettings {
        let setting = AVCapturePhotoSettings(format: [AVVideoCodecKey: AVVideoCodecType.hevc])
        setting.isDepthDataDeliveryEnabled = photoOutput.isDepthDataDeliverySupported
        return setting
    }
    
//  MARK: AutoCapture with Firebase Observer
    @IBAction func capture(_ state: UIButton, forEevent event: UISwitch){
        if (autoSavingSwitch.isOn && StateDeterminator){
//            self.startQueuedUpdates()
            self.timer = Timer.scheduledTimer(withTimeInterval: 5, repeats: true, block: { _ in
                if (self.autoSavingSwitch.isOn && self.StateDeterminator){
                    self.photoOutput.capturePhoto(with: self.getSettings(), delegate: self)
//                    self.startQueuedUpdates()
//      MARK: Sensory Information Pack and Send
//                    self.stopQueueUpdates()
//                    self.resetSensorData()
//                    self.resetLineData()
////                    sleep(1)
//                    self.startQueuedUpdates()
                }
            })
            UIImpactFeedbackGenerator(style: .light).impactOccurred()
            self.cameraView.layer.opacity = 0
            UIView.animate(withDuration: 0.25) {
            self.cameraView.layer.opacity = 1
            }
        }
        else{
//            self.writeSensorData()
            self.stopQueueUpdates()
            self.resetLineData()
            self.resetSensorData()
            return}
    }
//        sessionQueue.async {
//            self.photoOutput.capturePhoto(with: self.getSettings(), delegate: self)
//        }
        //Photo animations

    
    
//    @IBAction func capture(_ sender: UISwitch) {
//        if savingEnabled{
//            self.timer = Timer.scheduledTimer(withTimeInterval: 5, repeats: true, block: { _ in
//                self.photoOutput.capturePhoto(with: self.getSettings(), delegate: self)
//            })
//        }
//    }
//            sessionQueue.async {
//                self.photoOutput.capturePhoto(with: self.getSettings(), delegate: self)
//            }
//            sleep(5)
    
    @objc
    func didEnterBackground(notification: NSNotification) {
        // Free up resources.
        processingQueue.async {
            self.photoDepthConverter.reset()
        }
    }
}
extension UIImage {
    public convenience init?(pixelBuffer: CVPixelBuffer) {
        var cgImage: CGImage?
        VTCreateCGImageFromCVPixelBuffer(pixelBuffer, options: nil, imageOut: &cgImage)

        if let cgImage = cgImage {
            self.init(cgImage: cgImage, scale: 1.0, orientation: Orientation.right)
        } else {
            return nil
        }
    }
}
