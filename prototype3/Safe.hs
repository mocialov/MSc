{-# LANGUAGE ForeignFunctionInterface #-}
module Safe where

import Foreign
import Foreign.C.Types
import Data.Vector.Storable
import qualified Data.Vector.Storable as V
import AI.HNN.Recurrent.Network
import AI.HNN.FF.Network
import Numeric.LinearAlgebra
import Data.List.Split
import Data.Matrix
import Data.Vector
import Data.List.Split

-- exported functions
foreign export ccall process_network_input :: Ptr CInt -> Ptr CInt -> Ptr CInt -> Ptr CInt -> Ptr Double -> Ptr Double -> Ptr Double -> Ptr (Ptr Double) -> IO ()
foreign export ccall train :: Ptr CInt -> IO ()

-- get 1st and 2nd elements of a tuple
get1st (a,_) = a
get2nd (_,a) = a

-- train neural network. Numeric.LinearAlgebra.fromList [...] (substitute ... with the encoding that is used to represent training data)
-- (training data) '-->' (class)
-- after training, function stores trained network in "smartNet.nn" file (the file is encoded - do not try to read it)
train :: Ptr CInt -> IO ()
train n = do 
    net <- (AI.HNN.FF.Network.createNetwork 6 [12] 4) :: IO (AI.HNN.FF.Network.Network Double)
    let samples = [   Numeric.LinearAlgebra.fromList [...] --> Numeric.LinearAlgebra.fromList [...]
                    , Numeric.LinearAlgebra.fromList [...] --> Numeric.LinearAlgebra.fromList [...]
                    , ....
                  ]
    let smartNet = trainNTimes 1000 0.8 AI.HNN.FF.Network.sigmoid sigmoid' net samples
    saveNetwork "smartNet.nn" smartNet

-- feeds provided inputs to a trained network that is stored in a file
feed2 :: Int -> [Double] -> [Double] -> [Double -> Double] -> IO (Data.Vector.Storable.Vector Double)
feed2 nodes_number weights inputs_ functions = do
    n <- AI.HNN.FF.Network.loadNetwork "smartNet.nn" :: IO (AI.HNN.FF.Network.Network Double)
    let inputs__ = Numeric.LinearAlgebra.fromList inputs_
    return (output n AI.HNN.FF.Network.sigmoid inputs__)

-- feeds provided inputs to provided network (provided network must be encoded as recurrent) 
feed :: Int -> [Double] -> [[Double]] -> [Double -> Double] -> IO (Data.Vector.Storable.Vector Double)
feed nodes_number weights inputs_ functions = do
    let numNeurons = 442
        numInputs  = 441
        thresholds = Prelude.replicate numNeurons 0.0
        inputs     = inputs_
        adj        = weights
    n <- AI.HNN.Recurrent.Network.createNetwork numNeurons numInputs adj thresholds :: IO (AI.HNN.Recurrent.Network.Network Double)
    output <- evalNet n inputs AI.HNN.Recurrent.Network.sigmoid
    return output

-- returns converted Haskell integer from C integer
peekInt :: Ptr CInt -> IO Int
peekInt = fmap fromIntegral . peek

-- currently function is not used as hnn package has only sigmoid activation - all activations are set to sigmoid
build_function_list :: [Double] -> [Double -> Double]
build_function_list (x:xs) | x == 0.0 = AI.HNN.Recurrent.Network.sigmoid : build_function_list xs
                           | otherwise = AI.HNN.Recurrent.Network.sigmoid : build_function_list xs

process_network_input :: Ptr CInt -> Ptr CInt -> Ptr CInt -> Ptr CInt -> Ptr Double -> Ptr Double -> Ptr Double -> Ptr (Ptr Double) -> IO ()
process_network_input atype n m nodes_number weights inputs functions result = do
    atype <- peekInt atype -- type of processing -> 0 - recurrent with provided inputs and the network; 1 - feedforward using stored network and provided input
    n <- peekInt n -- number of weights
    m <- peekInt m -- number of inputs
    nodes_number <- peekInt nodes_number -- number of nodes
    weights_ <- peekArray n weights -- extracting weights
    inputs_ <- peekArray m inputs -- extracting inputs
    let inputs__ = inputs_:[] -- put inputs into a list (required by the processing method)
    functions_ <- peekArray nodes_number functions -- extracting activation functions
    let functions_list = build_function_list functions_ -- build a list of activation functions (all sigmoids)
    res <- case (atype == 0) of
        True -> (feed (Prelude.length functions_) weights_ inputs__ functions_list) --used inputs__ before
        False -> (feed2 (Prelude.length functions_) weights_ inputs_ functions_list) --used inputs__ before
    -- return result together with the size of the result (recurrent network returns result for every node of the network)
    let aList = (V.toList res)
    let b = (fromIntegral (Prelude.length aList)):aList
    ptr <- newArray b
    poke result $ ptr